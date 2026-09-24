"""src/robot/auto_release.py: Auto Release state machine (Phase 2): align, home, play, return home.

Owner decisions (docs/spec/operating-modes.md, section 4): with the pink basket in the front camera,
the arm moves to the home pose keeping the gripper unchanged (the egg stays held), a recorded release
motion delivers the egg and opens the mouth, and the arm returns home. Reviewer additions: the base
first aligns to the basket with the Auto Catch controller (basket cx and bbox width), and playback is
time-scaled (RELEASE_PLAYBACK_SPEED), rate-limited per joint, and starts only after the base has been
sent zeros for the whole home phase. Every frame returns the base action and the arm pose to send;
the mode manager cancels the action on Stop (the loop then holds the arm and zeroes the base). Pure
and stdlib-only; the caller passes the time, the basket detection, and the last commanded pose.
"""

from collections.abc import Mapping
from dataclasses import dataclass, replace
from typing import Final, Literal

from robot.align import AlignGains, AlignState, AlignTarget, align_step, start_align, zero_base
from robot.arm_follow import approach_pose, within
from robot.config import (
    ALIGN_FULL_SPEED_ERROR_H,
    ARM_ENGAGE_SPEED_DEG_S,
    ARM_KEYS,
    ARM_ENGAGE_TOLERANCE_DEG,
    RELEASE_HOME_TIMEOUT_S,
    RELEASE_MAX_JOINT_STEP_DEG_PER_S,
)
from robot.vision.basket_size import BasketSizeClass
from robot.vision.config_vision import (
    RELEASE_TARGET_CX,
    RELEASE_TARGET_CY,
    RELEASE_TARGET_W,
    RELEASE_TOL_CX,
    RELEASE_TOL_W_FAR,
    RELEASE_TOL_W_NEAR,
)

Pose = Mapping[str, float]
ReleasePhase = Literal["idle", "align", "home", "play", "return_home"]
ReleaseOutcome = Literal["running", "play_started", "done", "lost", "align_timeout", "home_timeout",
                         "start_timeout"]
GRIPPER_KEY: Final = "arm_gripper.pos"
JOINT_KEYS: Final = tuple(key for key in ARM_KEYS if key != GRIPPER_KEY)  # every key but the gripper
# The basket target for the shared alignment controller: bbox width is the size measure.
RELEASE_TARGET: Final = AlignTarget(cx=RELEASE_TARGET_CX, cy=RELEASE_TARGET_CY, h=RELEASE_TARGET_W, size_attr="w",
                                    larger_is_farther=False)  # a wider basket is closer
RELEASE_GAINS: Final = AlignGains(full_speed_error_h=ALIGN_FULL_SPEED_ERROR_H, tol_cx=RELEASE_TOL_CX,
                                  tol_h_far=RELEASE_TOL_W_FAR, tol_h_near=RELEASE_TOL_W_NEAR)


@dataclass(frozen=True)
class ReleaseLimits:
    """Arm tunables; the defaults come from config.py."""

    approach_speed: float = ARM_ENGAGE_SPEED_DEG_S
    tolerance: float = ARM_ENGAGE_TOLERANCE_DEG
    home_timeout_s: float = RELEASE_HOME_TIMEOUT_S
    max_joint_step_deg_s: float = RELEASE_MAX_JOINT_STEP_DEG_PER_S


DEFAULT_LIMITS: Final = ReleaseLimits()


@dataclass(frozen=True)
class ReleaseRequest:
    """What the Thx check needs: the basket size class and the loaded data (None = not recorded)."""

    size: BasketSizeClass = "none"
    home: Pose | None = None
    frames: tuple[Pose, ...] | None = None  # the motion resampled for playback, one pose per frame


@dataclass(frozen=True)
class ReleaseState:
    """Phase and its start time, the basket alignment, home pose, playback frames, the playback
    index (-1 while approaching the first frame), and the time of the last step."""

    phase: ReleasePhase = "idle"
    started_at: float = 0.0
    align: AlignState = AlignState()
    home: Pose | None = None
    frames: tuple[Pose, ...] = ()
    index: int = -1
    updated_at: float = 0.0


@dataclass(frozen=True)
class ReleaseStep:
    """One frame's result: next state, base action, arm pose to send, and the outcome."""

    state: ReleaseState
    base: dict[str, float]
    arm: dict[str, float]
    outcome: ReleaseOutcome


def start_release(now: float, home: Pose, frames: tuple[Pose, ...]) -> ReleaseState:
    """A fresh release in the align phase (base at rest, basket not yet smoothed)."""
    return ReleaseState(phase="align", started_at=now, align=start_align(now), home=dict(home),
                        frames=tuple(dict(pose) for pose in frames), updated_at=now)


def release_progress(state: ReleaseState) -> float | None:
    """Playback progress 0..1 while the recorded motion plays, else None."""
    if state.phase != "play" or state.index < 0 or not state.frames:
        return None
    return min(1.0, state.index / max(1, len(state.frames) - 1))


def _terminal(outcome: ReleaseOutcome, arm: Mapping[str, float]) -> ReleaseStep:
    return ReleaseStep(ReleaseState(), zero_base(), dict(arm), outcome)


def _next_phase(state: ReleaseState, phase: ReleasePhase, now: float) -> ReleaseState:
    return replace(state, phase=phase, started_at=now, index=-1, updated_at=now)


def _align(state: ReleaseState, basket: object | None, commanded: Pose, now: float) -> ReleaseStep:
    align, base, result = align_step(state.align, basket, now, RELEASE_TARGET, RELEASE_GAINS)
    if result == "running":
        return ReleaseStep(replace(state, align=align, updated_at=now), base, dict(commanded), "running")
    if result == "done":
        return ReleaseStep(_next_phase(replace(state, align=AlignState()), "home", now), zero_base(),
                           dict(commanded), "running")
    return _terminal("lost" if result == "lost" else "align_timeout", commanded)


def _home(state: ReleaseState, commanded: Pose, now: float, dt: float, limits: ReleaseLimits) -> ReleaseStep:
    """Toward the home pose at the engagement speed, the gripper kept at its current value."""
    if now - state.started_at > limits.home_timeout_s:
        return _terminal("home_timeout", commanded)
    target = {**dict(state.home or {}), GRIPPER_KEY: float(commanded[GRIPPER_KEY])}
    moved = approach_pose(commanded, target, dt, limits.approach_speed)
    if within(moved, target, limits.tolerance):
        return ReleaseStep(_next_phase(state, "play", now), zero_base(), moved, "play_started")
    return ReleaseStep(replace(state, updated_at=now), zero_base(), moved, "running")


def _play(state: ReleaseState, commanded: Pose, now: float, dt: float, limits: ReleaseLimits) -> ReleaseStep:
    """Approach the first frame slowly if needed with the gripper kept at its current value (the
    egg stays held until the arm is in place), then one resampled frame per loop frame, every joint
    capped at max_joint_step_deg_s * dt; the gripper follows the recording only from the first real
    playback frame on. The last frame (mouth open) is reached exactly before the arm returns home."""
    first, last_index = state.frames[0], len(state.frames) - 1
    if state.index < 0 and not within(commanded, first, limits.tolerance, JOINT_KEYS):
        if now - state.started_at > limits.home_timeout_s:
            return _terminal("start_timeout", commanded)
        target = {**dict(first), GRIPPER_KEY: float(commanded[GRIPPER_KEY])}
        moved = approach_pose(commanded, target, dt, limits.approach_speed)
        return ReleaseStep(replace(state, updated_at=now), zero_base(), moved, "running")
    index = min(max(state.index, 0), last_index)
    target = state.frames[index]
    if state.index < 0:  # transition frame: joints start the recording, the gripper is still held
        target = {**dict(target), GRIPPER_KEY: float(commanded[GRIPPER_KEY])}
    moved = approach_pose(commanded, target, dt, limits.max_joint_step_deg_s)
    if index == last_index and within(moved, target, 0.0):
        return ReleaseStep(_next_phase(state, "return_home", now), zero_base(), moved, "running")
    next_index = min(index + 1, last_index)
    return ReleaseStep(replace(state, index=next_index, updated_at=now), zero_base(), moved, "running")


def _return_home(state: ReleaseState, commanded: Pose, now: float, dt: float, limits: ReleaseLimits) -> ReleaseStep:
    """Back to the full home pose (gripper included) at the engagement speed."""
    if now - state.started_at > limits.home_timeout_s:
        return _terminal("home_timeout", commanded)
    home = dict(state.home or {})
    moved = approach_pose(commanded, home, dt, limits.approach_speed)
    if within(moved, home, limits.tolerance):
        return _terminal("done", moved)
    return ReleaseStep(replace(state, updated_at=now), zero_base(), moved, "running")


def release_step(
    state: ReleaseState,
    basket: object | None,
    commanded: Pose,
    now: float,
    limits: ReleaseLimits = DEFAULT_LIMITS,
) -> ReleaseStep:
    """One frame of Auto Release. `basket` is the newest basket detection (duck-typed cx, w) and
    `commanded` the last arm pose sent. Terminal outcomes return an idle state, zero base
    velocities, and the arm pose to hold; an idle state holds the arm and changes nothing."""
    dt = now - state.updated_at
    if state.phase == "align":
        return _align(state, basket, commanded, now)
    if state.phase == "home":
        return _home(state, commanded, now, dt, limits)
    if state.phase == "play" and state.frames:
        return _play(state, commanded, now, dt, limits)
    if state.phase == "return_home":
        return _return_home(state, commanded, now, dt, limits)
    return ReleaseStep(state, zero_base(), dict(commanded), "running")
