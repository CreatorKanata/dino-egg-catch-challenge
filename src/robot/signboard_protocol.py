"""src/robot/signboard_protocol.py: Pipe formats between Manual Mode and the signboard process.

Parent -> child (stdin): each packet is one UTF-8 JSON header line (drive, controller, mode
status with notice level and overlays, frame sizes) followed by the raw HWC uint8 BGR bytes of
every frame that has a signal, in header order. Child -> parent (stdout): one JSON line per
KachiButton command (or "capture"). The signboard runs in its own interpreter (opencv and
pygame each bundle SDL2 on macOS), so this module uses only the stdlib and numpy and is safe to
import on both sides.
"""

from dataclasses import asdict, dataclass
import json
import math
from typing import Any, BinaryIO

import numpy as np

from robot.config import SIGNBOARD_PROTOCOL_VERSION
from robot.dino_controller_reader import ControllerState
from robot.display_status import ARM_STATUSES, OVERLAY_KINDS, DisplayStatus, Overlay
from robot.drive_state import DriveState
from robot.mode_manager import ACTIONS, MODES, NOTICE_LEVELS

# Protocol bounds (not tunables): a header is a few hundred bytes; frames are camera-sized.
HEADER_MAX_BYTES = 64 * 1024
FRAME_SIDE_MAX = 8192
CHANNELS = 3
DRIVE_FIELDS = ("speed_index", "catch_requested", "input_lost", "pending_rotation_deg")
CONTROLLER_FIELDS = ("up", "down", "left", "right", "button", "synchronized")
STATUS_FIELDS = ("mode", "action", "voice_listening", "notice", "arm_status", "stopped", "notice_level")
OVERLAY_NUMBERS = ("cx", "cy", "w", "h", "angle")
# Normalized overlay bounds: a fitted ellipse of a partly visible egg may extend past the frame.
OVERLAY_BOUNDS = {"cx": (-1.0, 2.0), "cy": (-1.0, 2.0), "w": (0.0, 3.0), "h": (0.0, 3.0), "angle": (-360.0, 360.0)}
NOTICE_MAX_CHARS = 200
OVERLAYS_MAX = 8
OVERLAY_TEXT_MAX_CHARS = 64
COMMAND_MAX_CHARS = 64
COMMAND_LINE_MAX_BYTES = 1024


@dataclass(frozen=True)
class FramePacket:
    """Header entry for one camera; width = height = 0 means no signal."""

    name: str
    width: int
    height: int


@dataclass(frozen=True, eq=False)  # frames hold ndarrays, which have no scalar equality
class DisplayPacket:
    """Everything the signboard draws for one control-loop frame."""

    drive: DriveState
    controller: ControllerState
    frames: tuple[tuple[str, Any | None], ...]
    status: DisplayStatus = DisplayStatus()


@dataclass(frozen=True)
class CloseRequest:
    """Asks the signboard process to exit cleanly."""


def _as_frame(name: str, frame: Any | None) -> tuple[FramePacket, bytes]:
    if frame is None:
        return FramePacket(name, 0, 0), b""
    array = np.ascontiguousarray(frame, dtype=np.uint8)
    if array.ndim != 3 or array.shape[2] != CHANNELS or 0 in array.shape:
        raise ValueError(f"Frame {name!r} must be HxWx3, got shape {array.shape}")
    height, width = array.shape[:2]
    return FramePacket(name, width, height), array.tobytes()


def encode(packet: DisplayPacket | CloseRequest) -> bytes:
    """Serialize a packet: JSON header line, then the frame bytes."""
    if isinstance(packet, CloseRequest):
        header: dict[str, Any] = {"v": SIGNBOARD_PROTOCOL_VERSION, "type": "close"}
        return json.dumps(header).encode("utf-8") + b"\n"
    encoded = [_as_frame(name, frame) for name, frame in packet.frames]
    header = {
        "v": SIGNBOARD_PROTOCOL_VERSION,
        "drive": {field: getattr(packet.drive, field) for field in DRIVE_FIELDS},
        "controller": {field: getattr(packet.controller, field) for field in CONTROLLER_FIELDS},
        "status": {
            **{field: getattr(packet.status, field) for field in STATUS_FIELDS},
            "overlays": [asdict(overlay) for overlay in packet.status.overlays],
        },
        "frames": [{"name": entry.name, "w": entry.width, "h": entry.height} for entry, _ in encoded],
    }
    return json.dumps(header).encode("utf-8") + b"\n" + b"".join(body for _, body in encoded)


CLOSE_MESSAGE = encode(CloseRequest())


def encode_command(command: str) -> bytes:
    """One child -> parent line announcing a KachiButton command."""
    header = {"v": SIGNBOARD_PROTOCOL_VERSION, "type": "command", "command": command}
    return json.dumps(header).encode("utf-8") + b"\n"


def parse_command_line(line: bytes) -> str | None:
    """The command in one child -> parent line, or None when the line is malformed."""
    header = _parse_header(line)
    if header is None or header.get("type") != "command":
        return None
    command = header.get("command")
    if not isinstance(command, str) or not 0 < len(command) <= COMMAND_MAX_CHARS:
        return None
    return command


def _is_int(value: Any) -> bool:
    return type(value) is int


def _drive(fields: Any) -> DriveState | None:
    if not isinstance(fields, dict) or not _is_int(fields.get("speed_index")):
        return None
    if not all(type(fields.get(key)) is bool for key in ("catch_requested", "input_lost")):
        return None
    pending = fields.get("pending_rotation_deg")
    if type(pending) not in (int, float) or not math.isfinite(pending):
        return None
    return DriveState(**{key: fields[key] for key in DRIVE_FIELDS[:3]}, pending_rotation_deg=float(pending))


def _controller(fields: Any) -> ControllerState | None:
    if not isinstance(fields, dict) or not all(type(fields.get(key)) is bool for key in CONTROLLER_FIELDS):
        return None
    return ControllerState(**{key: fields[key] for key in CONTROLLER_FIELDS})


def _short_text(value: Any) -> bool:
    return isinstance(value, str) and len(value) <= OVERLAY_TEXT_MAX_CHARS


def _bounded(key: str, value: Any) -> bool:
    """A finite number within OVERLAY_BOUNDS[key]."""
    low, high = OVERLAY_BOUNDS[key]
    return type(value) in (int, float) and math.isfinite(value) and low <= value <= high


def _overlay(fields: Any) -> Overlay | None:
    if not isinstance(fields, dict) or fields.get("kind") not in OVERLAY_KINDS:
        return None
    if not (_short_text(fields.get("camera")) and _short_text(fields.get("label"))):
        return None
    if not all(_bounded(key, fields.get(key)) for key in OVERLAY_NUMBERS):
        return None
    numbers = {key: float(fields[key]) for key in OVERLAY_NUMBERS}
    return Overlay(camera=fields["camera"], kind=fields["kind"], label=fields["label"], **numbers)


def _overlays(entries: Any) -> tuple[Overlay, ...] | None:
    if not isinstance(entries, list) or len(entries) > OVERLAYS_MAX:
        return None
    overlays = tuple(map(_overlay, entries))
    return None if any(overlay is None for overlay in overlays) else overlays


def _status(fields: Any) -> DisplayStatus | None:
    if not isinstance(fields, dict):
        return None
    if not all(type(fields.get(key)) is bool for key in ("voice_listening", "stopped")):
        return None
    notice = fields.get("notice")
    if not isinstance(notice, str) or len(notice) > NOTICE_MAX_CHARS:
        return None
    if fields.get("mode") not in MODES or fields.get("action") not in ACTIONS:
        return None
    if fields.get("arm_status") not in ARM_STATUSES or fields.get("notice_level") not in NOTICE_LEVELS:
        return None
    overlays = _overlays(fields.get("overlays"))
    if overlays is None:
        return None
    return DisplayStatus(**{key: fields[key] for key in STATUS_FIELDS}, overlays=overlays)


def _frame_entry(entry: Any) -> FramePacket | None:
    if not isinstance(entry, dict) or not isinstance(entry.get("name"), str):
        return None
    width, height = entry.get("w"), entry.get("h")
    if not (_is_int(width) and _is_int(height)) or not (0 <= width <= FRAME_SIDE_MAX and 0 <= height <= FRAME_SIDE_MAX):
        return None
    if (width == 0) != (height == 0):
        return None
    return FramePacket(entry["name"], width, height)


def _read_exact(stream: BinaryIO, size: int) -> bytes | None:
    """Read exactly `size` bytes (works on raw pipes with short reads); None on EOF."""
    chunks, remaining = [], size
    while remaining > 0:
        chunk = stream.read(remaining)
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def _parse_header(line: bytes) -> dict[str, Any] | None:
    if not line.endswith(b"\n"):
        return None
    try:
        header = json.loads(line.decode("utf-8"))
    except (UnicodeDecodeError, ValueError):
        return None
    if not isinstance(header, dict) or header.get("v") != SIGNBOARD_PROTOCOL_VERSION:
        return None
    return header


def read_packet(stream: BinaryIO) -> DisplayPacket | CloseRequest | None:
    """Read one packet; None on EOF, a malformed header, or a short frame read."""
    header = _parse_header(stream.readline(HEADER_MAX_BYTES))
    if header is None:
        return None
    if "type" in header:
        return CloseRequest() if header["type"] == "close" else None
    drive, controller = _drive(header.get("drive")), _controller(header.get("controller"))
    status, entries = _status(header.get("status")), header.get("frames")
    if drive is None or controller is None or status is None or not isinstance(entries, list):
        return None
    frames = []
    for entry in map(_frame_entry, entries):
        if entry is None:
            return None
        frame = None
        if entry.width:
            data = _read_exact(stream, entry.width * entry.height * CHANNELS)
            if data is None:
                return None
            frame = np.frombuffer(data, dtype=np.uint8).reshape(entry.height, entry.width, CHANNELS)
        frames.append((entry.name, frame))
    return DisplayPacket(drive=drive, controller=controller, frames=tuple(frames), status=status)
