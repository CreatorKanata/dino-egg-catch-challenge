"""src/robot/dino_controller_reader.py: Receive dino-controller JSON v0 lines as immutable state.

Implements the PC receiver guidance of docs/dino-controller-protocol.md: LF framing with a
bounded buffer, strict v0 validation, snapshot synchronization, and sequence-gap handling.
The pure functions (feed, parse_line, apply) use only the stdlib so they are testable without
pyserial; SerialControllerReader imports `serial` lazily when a port is opened.
"""

from dataclasses import dataclass, replace
import json
import logging
import time
from typing import Any, Callable

from robot.config import (
    CONTROLLER_BAUD,
    CONTROLLER_INPUT_TIMEOUT_S,
    CONTROLLER_LINE_MAX_BYTES,
    CONTROLLER_STATE_POLL_INTERVAL_S,
)

logger = logging.getLogger(__name__)

PROTOCOL_VERSION = 0
DEVICE_NAME = "dino-controller"
SEQ_MODULUS = 2**32
STATE_COMMAND = b"STATE\n"
DIRECTIONS = ("up", "down", "left", "right")
ENCODER_DELTAS = {"cw": 1, "ccw": -1}
KNOWN_TYPES = frozenset({"ready", "state", "joystick", "encoder", "button", "error"})


@dataclass(frozen=True)
class ControllerState:
    """Latest cached controller levels; only meaningful while `synchronized` is True."""

    up: bool = False
    down: bool = False
    left: bool = False
    right: bool = False
    button: bool = False
    synchronized: bool = False
    last_seq: int | None = None
    last_update_monotonic: float | None = None


@dataclass(frozen=True)
class ParsedMessage:
    """One validated v0 message. Payload fields not used by `type` keep their defaults."""

    type: str
    seq: int
    ms: int
    up: bool = False
    down: bool = False
    left: bool = False
    right: bool = False
    button: bool = False
    delta: int = 0
    code: str = ""


INITIAL_STATE = ControllerState()


# --- Framing ---------------------------------------------------------------------------------

def feed(buffer: bytes, chunk: bytes, max_bytes: int = CONTROLLER_LINE_MAX_BYTES) -> tuple[bytes, tuple[bytes, ...]]:
    """Split buffered bytes into complete LF-terminated lines (trailing CR stripped).

    The remainder is capped at `max_bytes`; a capped remainder can never form a valid line,
    so an oversized line is discarded through its next LF while memory stays bounded.
    """
    parts = (buffer + chunk).split(b"\n")
    lines = tuple(_strip_cr(part) for part in parts[:-1] if len(part) + 1 <= max_bytes)
    return parts[-1][:max_bytes], lines


def _strip_cr(line: bytes) -> bytes:
    return line[:-1] if line.endswith(b"\r") else line


# --- Validation ------------------------------------------------------------------------------

def _is_bool(value: Any) -> bool:
    return type(value) is bool


def _is_u32(value: Any) -> bool:
    return type(value) is int and 0 <= value < SEQ_MODULUS


def _has_bools(obj: dict, keys: tuple[str, ...]) -> bool:
    return all(_is_bool(obj.get(key)) for key in keys)


def _valid_envelope(obj: dict) -> bool:
    version = obj.get("v")
    return (
        type(version) is int
        and version == PROTOCOL_VERSION
        and obj.get("type") in KNOWN_TYPES
        and _is_u32(obj.get("seq"))
        and _is_u32(obj.get("ms"))
    )


def _directions(obj: dict) -> dict[str, bool]:
    return {key: obj[key] for key in DIRECTIONS}


def _payload(obj: dict) -> dict[str, Any] | None:
    """Return the typed payload fields for a message, or None when the payload is invalid."""
    kind = obj["type"]
    if kind == "ready":
        return {}
    if kind == "state":
        valid = (obj.get("device") == DEVICE_NAME and _has_bools(obj, DIRECTIONS + ("button",))
                 and type(obj.get("ab")) is int and 0 <= obj["ab"] <= 3)
        return {**_directions(obj), "button": obj["button"]} if valid else None
    if kind == "joystick":
        return _directions(obj) if _has_bools(obj, DIRECTIONS) else None
    if kind == "button":
        return {"button": obj["pressed"]} if _is_bool(obj.get("pressed")) else None
    if kind == "encoder":
        expected = ENCODER_DELTAS.get(obj.get("direction"))
        delta = obj.get("delta")
        valid = expected is not None and type(delta) is int and delta == expected
        return {"delta": delta} if valid else None
    return {"code": obj["code"]} if isinstance(obj.get("code"), str) else None


def parse_line(line: bytes) -> ParsedMessage | None:
    """Parse one framed line; None for boot text, malformed JSON, or any invalid v0 field."""
    try:
        obj = json.loads(line.decode("ascii"))
    except (UnicodeDecodeError, ValueError):
        return None
    if not isinstance(obj, dict) or not _valid_envelope(obj):
        return None
    payload = _payload(obj)
    if payload is None:
        return None
    return ParsedMessage(type=obj["type"], seq=obj["seq"], ms=obj["ms"], **payload)


# --- State machine ---------------------------------------------------------------------------

def _cleared(state: ControllerState) -> ControllerState:
    """Drop held inputs and require a fresh snapshot, keeping sequence bookkeeping."""
    return ControllerState(last_seq=state.last_seq, last_update_monotonic=state.last_update_monotonic)


def _is_gap(state: ControllerState, seq: int) -> bool:
    return state.last_seq is not None and seq != (state.last_seq + 1) % SEQ_MODULUS


def apply(state: ControllerState, msg: ParsedMessage, now: float) -> tuple[ControllerState, int]:
    """Return the next state and the encoder delta to apply once (0 when none)."""
    gap = _is_gap(state, msg.seq)
    tracked = replace(state, last_seq=msg.seq, last_update_monotonic=now)
    if msg.type in ("ready", "error"):
        return _cleared(tracked), 0
    if gap:
        level = logging.WARNING if state.synchronized else logging.DEBUG
        logger.log(level, "Controller sequence gap (%s -> %s); resynchronizing", state.last_seq, msg.seq)
        tracked = _cleared(tracked)
    if msg.type == "state":
        levels = {key: getattr(msg, key) for key in DIRECTIONS + ("button",)}
        return replace(tracked, synchronized=True, **levels), 0
    if not tracked.synchronized:
        return tracked, 0
    if msg.type == "joystick":
        return replace(tracked, **{key: getattr(msg, key) for key in DIRECTIONS}), 0
    if msg.type == "button":
        return replace(tracked, button=msg.button), 0
    return tracked, msg.delta


# --- Serial port wrapper ---------------------------------------------------------------------

def _open_pyserial(**kwargs: Any) -> Any:
    import serial  # Lazy: keeps the pure functions importable without pyserial.

    return serial.Serial(**kwargs)


class SerialControllerReader:
    """Non-blocking reader that turns the controller's serial stream into ControllerState.

    `state` is only ever reassigned to new frozen instances. `STATE` is requested on open and
    every CONTROLLER_STATE_POLL_INTERVAL_S, because the firmware is silent while inputs are
    unchanged and a reply is needed after any desynchronization.
    """

    def __init__(
        self,
        port: str,
        baud: int = CONTROLLER_BAUD,
        timeout_s: float = CONTROLLER_INPUT_TIMEOUT_S,
        poll_interval_s: float = CONTROLLER_STATE_POLL_INTERVAL_S,
        serial_factory: Callable[..., Any] = _open_pyserial,
    ) -> None:
        self._port = port
        self._baud = baud
        self._timeout_s = timeout_s
        self._poll_interval_s = poll_interval_s
        self._serial_factory = serial_factory
        self._serial: Any = None
        self._buffer = b""
        self._last_request: float | None = None
        self.state = INITIAL_STATE

    def open(self, now: float | None = None) -> None:
        """Open the port, clear cached input and partial lines, then request a snapshot."""
        self._serial = self._serial_factory(port=self._port, baudrate=self._baud, timeout=0)
        self._serial.reset_input_buffer()
        self._buffer = b""
        self.state = INITIAL_STATE
        self.request_state(time.monotonic() if now is None else now)
        logger.info("Controller port %s opened at %d baud", self._port, self._baud)

    def request_state(self, now: float | None = None) -> None:
        """Send the plain-text STATE command (LF-terminated)."""
        self._require_open().write(STATE_COMMAND)
        self._last_request = time.monotonic() if now is None else now

    def poll(self, now: float) -> tuple[ControllerState, int]:
        """Drain available bytes and return the new state plus the summed encoder delta."""
        port = self._require_open()
        chunk = port.read(port.in_waiting) if port.in_waiting else b""
        self._buffer, lines = feed(self._buffer, chunk)
        state, total = self.state, 0
        for message in filter(None, map(parse_line, lines)):
            state, delta = apply(state, message, now)
            total += delta
        self.state = state
        if self._last_request is None or now - self._last_request >= self._poll_interval_s:
            self.request_state(now)
        return state, total

    def is_stale(self, now: float) -> bool:
        """True when no valid controller message arrived within the input timeout."""
        last = self.state.last_update_monotonic
        return last is None or now - last > self._timeout_s

    def close(self) -> None:
        """Close the port and forget held inputs; a fresh snapshot is required on reopen."""
        if self._serial is not None:
            self._serial.close()
            logger.info("Controller port %s closed", self._port)
        self._serial = None
        self._buffer = b""
        self.state = INITIAL_STATE

    def _require_open(self) -> Any:
        if self._serial is None:
            raise RuntimeError("Controller port is not open; call open() first")
        return self._serial
