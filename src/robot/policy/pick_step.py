"""src/robot/policy/pick_step.py: Per-frame safety guard and stop condition of the pick policy.

Auto Catch's `pick` phase (auto_catch.py) asks the policy runner for one action per loop frame.
This module decides what of it reaches the robot and when the phase ends: pick_guard caps every
arm key at max_step_deg_s * dt from the commanded pose (the release playback's cap, with the same
frame-time clamp) and keeps the base at zero whatever the policy outputs; pick_result ends the
phase with "done" once the proposals have settled with the mouth closed after a minimum time
(a proposal to tune on the robot; whether the egg is really held is not checked), or "timeout".
Pure and stdlib-only; the caller passes the time.
"""

from collections.abc import Mapping
from dataclasses import dataclass, replace
import math
from typing import Final, Literal

from robot.arm_follow import approach_pose
from robot.auto_release import GRIPPER_KEY
from robot.config import ARM_KEYS
from robot.controller_to_action import BASE_KEYS
from robot.policy.config_policy import (
    PICK_GRIPPER_CLOSED_MAX,
    PICK_MAX_JOINT_STEP_DEG_PER_S,
    PICK_MAX_S,
    PICK_MIN_S,
    PICK_SETTLE_DEG,
    PICK_SETTLE_FRAMES,
)

Pose = Mapping[str, float]
PickResult = Literal["running", "done", "timeout"]


@dataclass(frozen=True)
class PickLimits:
    """Time limits, stop condition, and joint cap; the defaults come from config_policy.py."""

    max_s: float = PICK_MAX_S
    min_s: float = PICK_MIN_S
    settle_frames: int = PICK_SETTLE_FRAMES
    settle_deg: float = PICK_SETTLE_DEG
    gripper_closed_max: float = PICK_GRIPPER_CLOSED_MAX
    max_step_deg_s: float = PICK_MAX_JOINT_STEP_DEG_PER_S


DEFAULT_PICK_LIMITS: Final = PickLimits()


@dataclass(frozen=True)
class PickState:
    """Start time, frames run, consecutive settled frames, the duration of the last policy call in
    ms, and the previous proposed arm pose (None before the first proposal)."""

    started_at: float = 0.0
    frames: int = 0
    settle_frames: int = 0
    last_inference_ms: float = 0.0
    previous: Pose | None = None


def start_pick(now: float) -> PickState:
    """A fresh pick phase starting at `now`."""
    return PickState(started_at=now)


def proposed_arm(proposed: Mapping[str, object]) -> dict[str, float]:
    """The six arm keys of a policy action as floats; ValueError names a missing or non-finite key."""
    missing = [key for key in ARM_KEYS if key not in proposed]
    if missing:
        raise ValueError(f"Policy action lacks {', '.join(missing)}")
    arm = {key: float(proposed[key]) for key in ARM_KEYS}  # type: ignore[arg-type]
    bad = [key for key, value in arm.items() if not math.isfinite(value)]
    if bad:
        raise ValueError(f"Policy action is not finite for {', '.join(bad)}")
    return arm


def pick_guard(commanded: Pose, proposed: Mapping[str, object], dt: float, max_step_deg_s: float) -> dict[str, float]:
    """The action to send: every arm key moved from `commanded` toward the proposal by at most
    max_step_deg_s * dt (dt clamped to [0, 2 / LOOP_HZ], as the playback), base keys at zero."""
    moved = approach_pose(commanded, proposed_arm(proposed), dt, max_step_deg_s)
    return {**moved, **{key: 0.0 for key in BASE_KEYS}}


def pick_timed_out(state: PickState, now: float, limits: PickLimits = DEFAULT_PICK_LIMITS) -> bool:
    return now - state.started_at >= limits.max_s


def _settle_count(state: PickState, arm: Pose, limits: PickLimits) -> int:
    """Consecutive frames, this one included, whose proposal moved less than settle_deg on every key."""
    if state.previous is None:
        return 0
    settled = all(abs(arm[key] - float(state.previous[key])) < limits.settle_deg for key in ARM_KEYS)
    return state.settle_frames + 1 if settled else 0


def advance_pick(state: PickState, proposed: Mapping[str, object], inference_ms: float,
                 limits: PickLimits = DEFAULT_PICK_LIMITS) -> PickState:
    """The state after this frame's proposal (a new object; `state` is unchanged)."""
    arm = proposed_arm(proposed)
    return replace(state, frames=state.frames + 1, settle_frames=_settle_count(state, arm, limits),
                   last_inference_ms=float(inference_ms), previous=arm)


def pick_result(state: PickState, commanded: Pose, proposed: Mapping[str, object], now: float,
                limits: PickLimits = DEFAULT_PICK_LIMITS) -> PickResult:
    """This frame's result from the state before it, the guarded command sent, and the proposal:
    "timeout" at max_s; "done" when the proposals have been settled for settle_frames frames in a
    row, the gripper command sent is at or below gripper_closed_max, and min_s has elapsed."""
    if pick_timed_out(state, now, limits):
        return "timeout"
    settled = _settle_count(state, proposed_arm(proposed), limits) >= limits.settle_frames
    closed = float(commanded[GRIPPER_KEY]) <= limits.gripper_closed_max
    return "done" if settled and closed and now - state.started_at >= limits.min_s else "running"
