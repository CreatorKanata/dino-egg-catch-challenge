"""src/robot/arm_follow.py: Slow engagement, then real-time following of the leader arm.

Owner decision (docs/spec/operating-modes.md, section 3): when Manual Mode starts or after a stop,
the dinosaur arm moves toward the leader's pose at a limited rate per key, and follows the
leader directly only once every key is within tolerance. This removes the jump a mismatched
leader would cause. approach_pose (per key, for the moving leader target and the capped
recorded-motion playback) and within are shared with Auto Release. approach_pose_sync moves to a
fixed pose along the straight line in joint space so every key arrives together (every scripted
pose move in Auto Catch and Auto Release): with per-key limits a small shoulder delta finished long
before a large wrist delta and the head first rose, then tilted down (robot run 2026-09-25). Pure
and stdlib-only; the caller passes the frame time.
"""

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
import math

from robot.config import ARM_ENGAGE_SPEED_DEG_S, ARM_ENGAGE_TOLERANCE_DEG, ARM_KEYS, LOOP_HZ

MAX_FRAME_DT_S = 2 / LOOP_HZ  # same clamp as the encoder rotation budget in drive_state.py


@dataclass(frozen=True)
class ArmFollowState:
    """False while syncing toward the leader, True once real-time following has begun."""

    engaged: bool = False


def disengaged() -> ArmFollowState:
    """Fresh state used on mode switches and Stop: following resumes through slow engagement."""
    return ArmFollowState(engaged=False)


def _approach(current: float, target: float, max_step: float) -> float:
    """Move `current` toward `target` by at most `max_step`, without overshooting."""
    delta = target - current
    return target if abs(delta) <= max_step else current + math.copysign(max_step, delta)


def approach_pose(
    current: Mapping[str, float],
    target: Mapping[str, float],
    dt: float,
    speed: float = ARM_ENGAGE_SPEED_DEG_S,
    keys: Iterable[str] = ARM_KEYS,
) -> dict[str, float]:
    """A new pose with every key of `keys` moved from `current` toward `target` by at most
    `speed * dt` (dt clamped to [0, 2 / LOOP_HZ]), without overshooting."""
    max_step = speed * min(max(dt, 0.0), MAX_FRAME_DT_S)
    return {key: _approach(float(current[key]), float(target[key]), max_step) for key in keys}


def approach_pose_sync(
    current: Mapping[str, float],
    target: Mapping[str, float],
    dt: float,
    speed: float = ARM_ENGAGE_SPEED_DEG_S,
    keys: Iterable[str] = ARM_KEYS,
) -> dict[str, float]:
    """A new pose (all ARM_KEYS) with the `keys` moved together along the straight line from `current`
    to `target`: the key with the largest delta D moves by at most `speed * dt` (dt clamped to
    [0, 2 / LOOP_HZ]), every other key by the same fraction of its own delta, so all arrive in the
    same frame. D == 0 returns the target. Keys not in `keys` (a held gripper) keep their value."""
    moving = tuple(keys)
    deltas = {key: float(target[key]) - float(current[key]) for key in moving}
    largest = max((abs(delta) for delta in deltas.values()), default=0.0)
    step = speed * min(max(dt, 0.0), MAX_FRAME_DT_S)
    fraction = 1.0 if largest == 0.0 else min(1.0, step / largest)
    moved = {key: float(target[key]) if fraction == 1.0 else float(current[key]) + delta * fraction
             for key, delta in deltas.items()}
    return {key: moved.get(key, float(current[key])) for key in ARM_KEYS}


def within(
    current: Mapping[str, float],
    target: Mapping[str, float],
    tolerance: float = ARM_ENGAGE_TOLERANCE_DEG,
    keys: Iterable[str] = ARM_KEYS,
) -> bool:
    """True when every key of `keys` is within `tolerance` of the target."""
    return all(abs(float(current[key]) - float(target[key])) <= tolerance for key in keys)


def follow_step(
    commanded: Mapping[str, float],
    leader: Mapping[str, float] | None,
    state: ArmFollowState,
    dt: float,
    *,
    speed: float = ARM_ENGAGE_SPEED_DEG_S,
    tolerance: float = ARM_ENGAGE_TOLERANCE_DEG,
) -> tuple[dict[str, float], ArmFollowState]:
    """Return the arm pose to send this frame and the next follow state.

    `commanded` is the last pose sent to the robot. Without a leader pose the arm holds it. When
    engaged the leader pose is sent as is. Otherwise every key moves at most `speed * dt` (dt
    clamped to [0, 2 / LOOP_HZ]) toward the leader; the gripper (percent) uses the same number.
    """
    if leader is None:
        return dict(commanded), state
    target = {key: float(leader[key]) for key in ARM_KEYS}
    if state.engaged:
        return target, state
    moved = approach_pose(commanded, target, dt, speed)
    if within(moved, target, tolerance):
        return target, ArmFollowState(engaged=True)
    return moved, state
