"""src/robot/align.py: Image-based base alignment for Auto Catch and Auto Release (Phase 2).

Drives the base until the egg in the front image sits at the best position (ALIGN_TARGET_* in
config.py; docs/spec/operating-modes.md, section 4). LeKiwiClient's sign convention (forward =
+x, left = +y): an egg right of the target moves the base right (-y); an egg farther than the
target drives forward (+x). The egg's distance measure is its fitted-ellipse width
(EggDetection.ellipse_w; narrower = farther): the bbox height changes with the light (the shadowed
underside merges with the tarp) and the top edge saturates at the camera's horizon closer than the
best position (robot run 2026-09-25). Owner request (2026-09-24, the first version oscillated): the speed
tapers linearly to zero near the target (max_xy * clamp(error / full-speed error, -1, 1)), an axis
inside its tolerance gets 0 (the distance window is asymmetric: closer is fine, farther is not),
tiny commands become 0, cx and the distance measure are smoothed with an exponential moving
average, and each command changes by at most max_accel * dt per frame. While the egg
flickers out of detection (fewer than lost_frames consecutive misses) the controller keeps steering
toward the last smoothed position (robot run 2026-09-24: far eggs flicker). Terminal results
return zeros at once. The distance measure is selectable (AlignTarget.size_attr and
larger_is_farther): the egg uses its ellipse width, the Auto Release basket its bbox width (both
larger = closer; the same controller, other target and tolerances). Pure and stdlib-only:
detections are duck-typed (cx and the measure's attribute).
"""

from dataclasses import dataclass, replace
from typing import Literal, Protocol

from robot.config import (
    ALIGN_DONE_FRAMES,
    ALIGN_FULL_SPEED_ERROR_CX,
    ALIGN_FULL_SPEED_ERROR_W,
    ALIGN_LOST_FRAMES,
    ALIGN_MAX_ACCEL,
    ALIGN_MAX_XY,
    ALIGN_MIN_XY,
    ALIGN_SMOOTHING,
    ALIGN_TARGET_CX,
    ALIGN_TARGET_CY,
    ALIGN_TARGET_W_EGG,
    ALIGN_TIMEOUT_S,
    ALIGN_TOL_CX,
    ALIGN_TOL_W_FAR,
    ALIGN_TOL_W_NEAR,
    LOOP_HZ,
)

AlignPhase = Literal["idle", "aligning"]
AlignResult = Literal["running", "done", "lost", "timeout"]
MAX_FRAME_DT_S = 2 / LOOP_HZ  # same clamp as the rotation budget and the arm approach


class EggLike(Protocol):
    """The detection fields the egg path uses (normalized bbox center x, bbox width and height, top
    edge, ellipse width); the controller reads the attribute named by AlignTarget.size_attr (the
    basket path reads `w`)."""

    cx: float
    w: float
    h: float
    top: float
    ellipse_w: float


@dataclass(frozen=True)
class AlignTarget:
    """Best position in the front image, normalized: bbox center and the target value `h` of the
    distance measure named by `size_attr` (the egg's ellipse width by default; "w" for the basket).
    `larger_is_farther`: False when a larger value means closer (widths, height), True when it means
    farther (a measure like the top edge, which rises toward the horizon)."""

    cx: float = ALIGN_TARGET_CX
    cy: float = ALIGN_TARGET_CY
    h: float = ALIGN_TARGET_W_EGG
    size_attr: Literal["h", "w", "top", "ellipse_w"] = "ellipse_w"
    larger_is_farther: bool = False


@dataclass(frozen=True)
class AlignGains:
    """Controller tunables; the defaults come from config.py."""

    max_xy: float = ALIGN_MAX_XY
    full_speed_error_cx: float = ALIGN_FULL_SPEED_ERROR_CX
    full_speed_error_h: float = ALIGN_FULL_SPEED_ERROR_W  # of the distance measure
    min_xy: float = ALIGN_MIN_XY
    tol_cx: float = ALIGN_TOL_CX
    tol_h_far: float = ALIGN_TOL_W_FAR  # measure farther than the target by at most this
    tol_h_near: float = ALIGN_TOL_W_NEAR  # measure closer than the target by at most this
    smoothing: float = ALIGN_SMOOTHING
    max_accel: float = ALIGN_MAX_ACCEL
    done_frames: int = ALIGN_DONE_FRAMES
    lost_frames: int = ALIGN_LOST_FRAMES
    timeout_s: float = ALIGN_TIMEOUT_S


@dataclass(frozen=True)
class Measurement:
    """Smoothed position: normalized cx and `h`, the distance measure (egg ellipse width or basket width)."""

    cx: float
    h: float


@dataclass(frozen=True)
class AlignState:
    """Idle, or aligning since `started_at`: frame counters, smoothed egg, last command and time."""

    phase: AlignPhase = "idle"
    started_at: float = 0.0
    ok_frames: int = 0
    lost_frames: int = 0
    smoothed: Measurement | None = None
    last_x: float = 0.0
    last_y: float = 0.0
    updated_at: float = 0.0


DEFAULT_TARGET = AlignTarget()
DEFAULT_GAINS = AlignGains()


def zero_base() -> dict[str, float]:
    """Zero velocity for every base axis (same keys as controller_to_action.stop_action)."""
    return {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}


def start_align(now: float) -> AlignState:
    """A fresh aligning state started at `now` (no smoothed egg yet, base at rest)."""
    return AlignState(phase="aligning", started_at=now, updated_at=now)


def smooth(
    previous: Measurement | None, det: EggLike | None, weight: float = ALIGN_SMOOTHING, size_attr: str = "ellipse_w"
) -> Measurement | None:
    """EMA of cx and the size measure (attribute `size_attr`) with `weight` on the new sample; the
    first sample is taken as is and a missing detection leaves the average unchanged."""
    if det is None:
        return previous
    size = float(getattr(det, size_attr))
    if previous is None:
        return Measurement(cx=det.cx, h=size)
    return Measurement(cx=weight * det.cx + (1 - weight) * previous.cx,
                       h=weight * size + (1 - weight) * previous.h)


def _taper(error: float, tol_below: float, tol_above: float, full_speed_error: float, gains: AlignGains) -> float:
    """0 inside [-tol_below, tol_above]; else max_xy scaled by error / full_speed_error (clamped to
    +-1), and 0 when that is slower than min_xy."""
    if -tol_below <= error <= tol_above:
        return 0.0
    command = gains.max_xy * max(-1.0, min(1.0, error / full_speed_error))
    return 0.0 if abs(command) < gains.min_xy else command


def align_command(
    det: EggLike | None, target: AlignTarget = DEFAULT_TARGET, gains: AlignGains = DEFAULT_GAINS
) -> tuple[dict[str, float], bool]:
    """Target base velocities for a position and whether it is inside both tolerances.

    `det` is a detection or a Measurement; a Measurement's `h` already is the size measure, a
    detection's size is read from target.size_attr. Without a detection the target is zero and
    nothing is aligned.
    """
    if det is None:
        return zero_base(), False
    size = det.h if isinstance(det, Measurement) else float(getattr(det, target.size_attr))
    error_cx = det.cx - target.cx
    error_h = size - target.h if target.larger_is_farther else target.h - size  # > 0: farther -> forward
    base = {
        "x.vel": _taper(error_h, gains.tol_h_near, gains.tol_h_far, gains.full_speed_error_h, gains),
        "y.vel": _taper(-error_cx, gains.tol_cx, gains.tol_cx, gains.full_speed_error_cx, gains),
        "theta.vel": 0.0,
    }
    in_height = -gains.tol_h_near <= error_h <= gains.tol_h_far
    return base, abs(error_cx) <= gains.tol_cx and in_height


def rate_limit(previous: float, wanted: float, dt: float, max_accel: float = ALIGN_MAX_ACCEL) -> float:
    """Move from `previous` toward `wanted` by at most max_accel * dt (dt clamped to 2 / LOOP_HZ)."""
    max_step = max_accel * min(max(dt, 0.0), MAX_FRAME_DT_S)
    return previous + max(-max_step, min(max_step, wanted - previous))


def align_step(
    state: AlignState,
    det: EggLike | None,
    now: float,
    target: AlignTarget = DEFAULT_TARGET,
    gains: AlignGains = DEFAULT_GAINS,
) -> tuple[AlignState, dict[str, float], AlignResult]:
    """One frame of alignment: the next state, the base action, and the result.

    Timeout is checked first, then egg loss (consecutive frames without a detection; until then the
    command keeps following the last smoothed position and the in-tolerance count is kept, neither
    advanced nor reset), then alignment on the smoothed egg (consecutive in-tolerance frames;
    drifting out resets the count). Terminal results return an idle state and zeros.
    """
    if now - state.started_at > gains.timeout_s:
        return AlignState(), zero_base(), "timeout"
    lost = state.lost_frames + 1 if det is None else 0
    if lost >= gains.lost_frames:
        return AlignState(), zero_base(), "lost"
    smoothed = smooth(state.smoothed, det, gains.smoothing, target.size_attr)
    wanted, done = align_command(smoothed, target, gains)
    if det is None:
        ok_frames = state.ok_frames  # a flicker neither advances nor resets the progress
    else:
        ok_frames = state.ok_frames + 1 if done else 0
    if ok_frames >= gains.done_frames:
        return AlignState(), zero_base(), "done"
    dt = now - state.updated_at
    x_vel = rate_limit(state.last_x, wanted["x.vel"], dt, gains.max_accel)
    y_vel = rate_limit(state.last_y, wanted["y.vel"], dt, gains.max_accel)
    next_state = replace(state, ok_frames=ok_frames, lost_frames=lost, smoothed=smoothed, last_x=x_vel,
                         last_y=y_vel, updated_at=now)
    return next_state, {"x.vel": x_vel, "y.vel": y_vel, "theta.vel": 0.0}, "running"


@dataclass(frozen=True)
class TraceRow:
    """One alignment frame for offline tuning: time since start, raw cx, distance measure (w_raw: the
    egg's ellipse width), bbox height and top edge (for analysis), smoothed cx and distance measure,
    command, result."""

    t: float
    cx_raw: float | None
    w_raw: float | None
    h_raw: float | None
    top_raw: float | None
    cx_smooth: float | None
    w_smooth: float | None
    x_vel: float
    y_vel: float
    result: AlignResult


def trace_row(
    before: AlignState, det: EggLike | None, now: float, base: dict[str, float], result: AlignResult,
    smoothing: float = ALIGN_SMOOTHING, target: AlignTarget = DEFAULT_TARGET,
) -> TraceRow:
    """The trace row for the frame align_step just computed from `before` and `det` (egg target)."""
    smoothed = smooth(before.smoothed, det, smoothing, target.size_attr)
    return TraceRow(
        t=now - before.started_at,
        cx_raw=None if det is None else det.cx,
        w_raw=None if det is None else float(getattr(det, target.size_attr)),
        h_raw=None if det is None else det.h,
        top_raw=None if det is None else getattr(det, "top", None),
        cx_smooth=None if smoothed is None else smoothed.cx,
        w_smooth=None if smoothed is None else smoothed.h,
        x_vel=base["x.vel"],
        y_vel=base["y.vel"],
        result=result,
    )
