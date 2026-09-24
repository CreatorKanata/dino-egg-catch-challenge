"""src/robot/signboard_protocol.py: Pipe format between Drive Mode and the signboard process.

Each packet is one UTF-8 JSON header line followed by the raw HWC uint8 BGR bytes of every
frame that has a signal, in header order. The signboard runs in its own interpreter (opencv
and pygame each bundle SDL2 on macOS), so this module uses only the stdlib and numpy and is
safe to import on both sides.
"""

from dataclasses import dataclass
import json
import math
from typing import Any, BinaryIO

import numpy as np

from robot.config import SIGNBOARD_PROTOCOL_VERSION
from robot.dino_controller_reader import ControllerState
from robot.drive_state import DriveState

# Protocol bounds (not tunables): a header is a few hundred bytes; frames are camera-sized.
HEADER_MAX_BYTES = 64 * 1024
FRAME_SIDE_MAX = 8192
CHANNELS = 3
DRIVE_FIELDS = ("speed_index", "catch_requested", "input_lost", "pending_rotation_deg")
CONTROLLER_FIELDS = ("up", "down", "left", "right", "button", "synchronized")


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
        "frames": [{"name": entry.name, "w": entry.width, "h": entry.height} for entry, _ in encoded],
    }
    return json.dumps(header).encode("utf-8") + b"\n" + b"".join(body for _, body in encoded)


CLOSE_MESSAGE = encode(CloseRequest())


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
    entries = header.get("frames")
    if drive is None or controller is None or not isinstance(entries, list):
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
    return DisplayPacket(drive=drive, controller=controller, frames=tuple(frames))
