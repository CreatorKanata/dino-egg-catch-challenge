"""src/robot/align.py: Image-based base alignment for Auto Catch (Phase 2, step 1).

Drives the base until the egg's bbox in the front image sits at the best position (ALIGN_TARGET_*
in config.py; docs/spec/operating-modes.md, section 4). Proportional control on two errors, with
LeKiwiClient's sign convention (forward = +x, left = +y): an egg right of the target moves the
base right (-y); an egg smaller than the target drives forward (+x). Speeds are clamped to the
slow level; an axis already inside its tolerance gets 0 (so "done" and "no motion" agree), and
tiny commands are zeroed so the base does not creep. Pure and stdlib-only: the
detection is duck-typed (cx, h), so no OpenCV is needed here.
"""

from dataclasses import dataclass
from typing import Literal, Protocol

from robot.config import (
    ALIGN_DONE_FRAMES,
    ALIGN_GAIN_X,
    ALIGN_GAIN_Y,
    ALIGN_LOST_FRAMES,
    ALIGN_MAX_XY,
    ALIGN_MIN_XY,
    ALIGN_TARGET_CX,
    ALIGN_TARGET_CY,
    ALIGN_TARGET_H,
    ALIGN_TIMEOUT_S,
    ALIGN_TOL_CX,
    ALIGN_TOL_H,
)

AlignPhase = Literal["idle", "aligning"]
AlignResult = Literal["running", "done", "lost", "timeout"]


class EggLike(Protocol):
    """The two detection fields the controller uses (normalized bbox center x and height)."""

    cx: float
    h: float


@dataclass(frozen=True)
class AlignTarget:
    """Best egg position in the front image, normalized: bbox center and bbox height."""

    cx: float = ALIGN_TARGET_CX
    cy: float = ALIGN_TARGET_CY
    h: float = ALIGN_TARGET_H


@dataclass(frozen=True)
class AlignGains:
    """Controller tunables; the defaults come from config.py."""

    gain_x: float = ALIGN_GAIN_X
    gain_y: float = ALIGN_GAIN_Y
    max_xy: float = ALIGN_MAX_XY
    min_xy: float = ALIGN_MIN_XY
    tol_cx: float = ALIGN_TOL_CX
    tol_h: float = ALIGN_TOL_H
    done_frames: int = ALIGN_DONE_FRAMES
    lost_frames: int = ALIGN_LOST_FRAMES
    timeout_s: float = ALIGN_TIMEOUT_S


@dataclass(frozen=True)
class AlignState:
    """Idle, or aligning since `started_at` with consecutive in-tolerance and egg-less frames."""

    phase: AlignPhase = "idle"
    started_at: float = 0.0
    ok_frames: int = 0
    lost_frames: int = 0


DEFAULT_TARGET = AlignTarget()
DEFAULT_GAINS = AlignGains()


def zero_base() -> dict[str, float]:
    """Zero velocity for every base axis (same keys as controller_to_action.stop_action)."""
    return {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}


def start_align(now: float) -> AlignState:
    """A fresh aligning state started at `now`."""
    return AlignState(phase="aligning", started_at=now)


def _shape(value: float, error: float, tolerance: float, gains: AlignGains) -> float:
    """0 inside the tolerance; else clamp to +-max_xy and zero anything slower than min_xy."""
    if abs(error) <= tolerance:
        return 0.0
    clamped = max(-gains.max_xy, min(gains.max_xy, value))
    return 0.0 if abs(clamped) < gains.min_xy else clamped


def align_command(
    det: EggLike | None, target: AlignTarget = DEFAULT_TARGET, gains: AlignGains = DEFAULT_GAINS
) -> tuple[dict[str, float], bool]:
    """Base velocities toward the target and whether the egg is inside both tolerances.

    Without a detection the base stays at zero and the egg is not aligned.
    """
    if det is None:
        return zero_base(), False
    error_cx = det.cx - target.cx
    error_h = target.h - det.h
    base = {
        "x.vel": _shape(gains.gain_x * error_h, error_h, gains.tol_h, gains),
        "y.vel": _shape(-gains.gain_y * error_cx, error_cx, gains.tol_cx, gains),
        "theta.vel": 0.0,
    }
    return base, abs(error_cx) <= gains.tol_cx and abs(error_h) <= gains.tol_h


def align_step(
    state: AlignState,
    det: EggLike | None,
    now: float,
    target: AlignTarget = DEFAULT_TARGET,
    gains: AlignGains = DEFAULT_GAINS,
) -> tuple[AlignState, dict[str, float], AlignResult]:
    """One frame of alignment: the next state, the base action, and the result.

    Timeout is checked first, then egg loss (consecutive frames without a detection; the base is
    at zero meanwhile), then alignment (consecutive in-tolerance frames; drifting out resets the
    count). Every terminal result returns an idle state and zero velocities.
    """
    if now - state.started_at > gains.timeout_s:
        return AlignState(), zero_base(), "timeout"
    if det is None:
        lost = state.lost_frames + 1
        if lost >= gains.lost_frames:
            return AlignState(), zero_base(), "lost"
        return AlignState("aligning", state.started_at, 0, lost), zero_base(), "running"
    base, done = align_command(det, target, gains)
    ok_frames = state.ok_frames + 1 if done else 0
    if ok_frames >= gains.done_frames:
        return AlignState(), zero_base(), "done"
    return AlignState("aligning", state.started_at, ok_frames, 0), base, "running"
