"""src/robot/arm_follow.py: Slow engagement, then real-time following of the leader arm.

Owner decision (docs/spec/operating-modes.md, section 3): when Manual Mode starts or after a stop,
the dinosaur arm moves toward the leader's pose at a limited rate per key, and follows the
leader directly only once every key is within tolerance. This removes the jump a mismatched
leader would cause. Pure and stdlib-only; the caller passes the frame time.
"""

from collections.abc import Mapping
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
    max_step = speed * min(max(dt, 0.0), MAX_FRAME_DT_S)
    moved = {key: _approach(float(commanded[key]), target[key], max_step) for key in ARM_KEYS}
    if all(abs(moved[key] - target[key]) <= tolerance for key in ARM_KEYS):
        return target, ArmFollowState(engaged=True)
    return moved, state
