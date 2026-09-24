"""src/robot/auto_catch.py: Auto Catch state machine (Phase 3 step 1): align, catch pose, wrist check, return.

Owner decisions (docs/spec/operating-modes.md, sections 1 and 4): after the base alignment the arm
moves slowly to the recorded catch pose (head down, the wrist camera sees the egg), the wrist view
must show the egg, the pick policy runs (a stub in this step: a notice, no motion), and the arm moves
slowly to the release pose (home_pose.json, head up) with the gripper kept, so a caught egg stays
held. Phases: [to_start] -> align -> to_catch -> wrist_check -> pick_stub -> to_release -> idle.
Owner decision (2026-09-25): an arm that the leader left off the release pose (any non-gripper joint
farther than POSE_NEAR_TOLERANCE_DEG) first moves there in `to_start` with the base at zero, because
driving with the head low or moving straight to the catch pose from an arbitrary pose could sweep
the head through the egg; release -> catch is the known-safe path (CATCH_VIA_RELEASE_POSE switches
it off for maintenance). Every scripted pose move is synchronized (all joints arrive together). A failed wrist
check or a catch-pose timeout warns and still returns to the release pose; the warning is shown
again when the arm is back ("Ready" only after a pass through the stub). Modeled on
auto_release.py and reusing its gripper key and pose timeout, the shared alignment controller, and
arm_follow's rate-limited, synchronized approach (all joints arrive together). Every frame returns the base action and the arm pose to send;
the mode manager cancels the action on Stop (the loop then holds the arm and zeroes the base). Pure
and stdlib-only; the caller passes the time, the egg and wrist detections, and the commanded pose.
"""

from collections.abc import Mapping
from dataclasses import dataclass, replace
from typing import Final, Literal

from robot.align import AlignResult, AlignState, align_step, start_align, zero_base
from robot.arm_follow import approach_pose_sync, within
from robot.auto_release import GRIPPER_KEY, JOINT_KEYS
from robot.config import (
    ARM_ENGAGE_SPEED_DEG_S,
    ARM_ENGAGE_TOLERANCE_DEG,
    CATCH_VIA_RELEASE_POSE,
    POSE_NEAR_TOLERANCE_DEG,
    RELEASE_HOME_TIMEOUT_S,
)
from robot.vision.config_vision import WRIST_CHECK_ENABLED, WRIST_CHECK_FRAMES
from robot.vision.egg_size import SizeClass

Pose = Mapping[str, float]
CatchPhase = Literal["idle", "to_start", "align", "to_catch", "wrist_check", "pick_stub", "to_release"]
CATCH_PHASES: Final = ("idle", "to_start", "align", "to_catch", "wrist_check", "pick_stub", "to_release")
# Terminal: start_timeout, lost, align_timeout, release_timeout, done, and the two *_returned (back at
# the release pose after a failure). The others only show a notice.
CatchFailure = Literal["", "catch_timeout", "no_wrist_egg"]
CatchOutcome = Literal["running", "start_timeout", "lost", "align_timeout", "catch_timeout", "no_wrist_egg",
                       "policy_stub", "release_timeout", "done", "catch_timeout_returned", "no_wrist_egg_returned"]
RETURNED: Final = {"catch_timeout": "catch_timeout_returned", "no_wrist_egg": "no_wrist_egg_returned"}


@dataclass(frozen=True)
class CatchLimits:
    """Arm and wrist-check tunables; the defaults come from config.py and config_vision.py."""

    approach_speed: float = ARM_ENGAGE_SPEED_DEG_S
    tolerance: float = ARM_ENGAGE_TOLERANCE_DEG
    pose_timeout_s: float = RELEASE_HOME_TIMEOUT_S
    wrist_check_frames: int = WRIST_CHECK_FRAMES
    wrist_check_enabled: bool = WRIST_CHECK_ENABLED
    via_release_pose: bool = CATCH_VIA_RELEASE_POSE
    near_tolerance: float = POSE_NEAR_TOLERANCE_DEG


DEFAULT_CATCH_LIMITS: Final = CatchLimits()


@dataclass(frozen=True)
class CatchRequest:
    """What the Hi! check needs: the egg size class, the recorded poses (None = not recorded), and the
    arm pose commanded now (None = unknown, treated as off the release pose)."""

    size: SizeClass = "none"
    catch: Pose | None = None
    home: Pose | None = None  # the release pose (home_pose.json)
    arm: Pose | None = None


@dataclass(frozen=True)
class CatchState:
    """Phase and its start time, the alignment, the two poses, the wrist-check frame count and
    whether an egg was seen in any of them, the time of the last step, and the failure that sent the
    arm back to the release pose ("" on the success path)."""

    phase: CatchPhase = "idle"
    started_at: float = 0.0
    align: AlignState = AlignState()
    catch: Pose | None = None
    home: Pose | None = None
    checked_frames: int = 0
    egg_seen: bool = False
    updated_at: float = 0.0
    failure: CatchFailure = ""


@dataclass(frozen=True)
class CatchStep:
    """One frame's result: next state, base action, arm pose to send, the outcome, and the
    alignment result when the frame was an alignment frame (for the trace), else None."""

    state: CatchState
    base: dict[str, float]
    arm: dict[str, float]
    outcome: CatchOutcome
    align_result: AlignResult | None = None


def start_catch(now: float, catch: Pose, home: Pose, arm: Pose | None,
                limits: CatchLimits = DEFAULT_CATCH_LIMITS) -> CatchState:
    """A fresh Auto Catch: in `to_start` when the arm is off the release pose (or unknown) and the
    rule is on, else in the align phase (base at rest, egg not yet smoothed)."""
    off = arm is None or not within(arm, home, limits.near_tolerance, JOINT_KEYS)
    phase: CatchPhase = "to_start" if limits.via_release_pose and off else "align"
    return CatchState(phase=phase, started_at=now, align=start_align(now), catch=dict(catch), home=dict(home),
                      updated_at=now)


def _terminal(outcome: CatchOutcome, arm: Pose, align_result: AlignResult | None = None) -> CatchStep:
    return CatchStep(CatchState(), zero_base(), dict(arm), outcome, align_result)


def _next_phase(state: CatchState, phase: CatchPhase, now: float) -> CatchState:
    return replace(state, phase=phase, started_at=now, checked_frames=0, egg_seen=False, updated_at=now)


def _hold(state: CatchState, commanded: Pose, outcome: CatchOutcome = "running") -> CatchStep:
    return CatchStep(state, zero_base(), dict(commanded), outcome)


def _align(state: CatchState, egg: object | None, commanded: Pose, now: float) -> CatchStep:
    align, base, result = align_step(state.align, egg, now)
    if result == "running":
        return CatchStep(replace(state, align=align, updated_at=now), base, dict(commanded), "running", result)
    if result == "done":
        moving = _next_phase(replace(state, align=AlignState()), "to_catch", now)
        return CatchStep(moving, zero_base(), dict(commanded), "running", result)
    return _terminal("lost" if result == "lost" else "align_timeout", commanded, result)


def _to_start(state: CatchState, commanded: Pose, now: float, dt: float, limits: CatchLimits) -> CatchStep:
    """Toward the release pose with the gripper held, base at zero; then a fresh alignment."""
    if now - state.started_at > limits.pose_timeout_s:
        return _terminal("start_timeout", commanded)
    target = {**dict(state.home or {}), GRIPPER_KEY: float(commanded[GRIPPER_KEY])}
    moved = approach_pose_sync(commanded, target, dt, limits.approach_speed, JOINT_KEYS)
    if within(moved, target, limits.tolerance):
        return CatchStep(replace(_next_phase(state, "align", now), align=start_align(now)), zero_base(), moved,
                         "running")
    return CatchStep(replace(state, updated_at=now), zero_base(), moved, "running")


def _to_catch(state: CatchState, commanded: Pose, now: float, dt: float, limits: CatchLimits) -> CatchStep:
    """Toward the full catch pose, the gripper included (the mouth opens as recorded)."""
    if now - state.started_at > limits.pose_timeout_s:
        return _hold(_next_phase(replace(state, failure="catch_timeout"), "to_release", now), commanded,
                     "catch_timeout")
    target = dict(state.catch or {})
    moved = approach_pose_sync(commanded, target, dt, limits.approach_speed)  # all keys arrive together
    if within(moved, target, limits.tolerance):
        return CatchStep(_next_phase(state, "wrist_check", now), zero_base(), moved, "running")
    return CatchStep(replace(state, updated_at=now), zero_base(), moved, "running")


def _wrist_check(state: CatchState, wrist: object | None, commanded: Pose, now: float,
                 limits: CatchLimits) -> CatchStep:
    """Hold the catch pose for wrist_check_frames frames; an egg in any of them passes. A disabled
    check always passes."""
    seen = state.egg_seen or wrist is not None
    checked = state.checked_frames + 1
    if checked < limits.wrist_check_frames:
        return _hold(replace(state, checked_frames=checked, egg_seen=seen, updated_at=now), commanded)
    if limits.wrist_check_enabled and not seen:
        return _hold(_next_phase(replace(state, failure="no_wrist_egg"), "to_release", now), commanded,
                     "no_wrist_egg")
    return _hold(_next_phase(state, "pick_stub", now), commanded)


def _to_release(state: CatchState, commanded: Pose, now: float, dt: float, limits: CatchLimits) -> CatchStep:
    """Toward the release pose (home) with the gripper kept at its current value."""
    if now - state.started_at > limits.pose_timeout_s:
        return _terminal("release_timeout", commanded)
    target = {**dict(state.home or {}), GRIPPER_KEY: float(commanded[GRIPPER_KEY])}
    moved = approach_pose_sync(commanded, target, dt, limits.approach_speed, JOINT_KEYS)  # gripper held
    if within(moved, target, limits.tolerance):
        return _terminal(RETURNED[state.failure] if state.failure else "done", moved)
    return CatchStep(replace(state, updated_at=now), zero_base(), moved, "running")


def catch_step(
    state: CatchState,
    egg: object | None,
    wrist: object | None,
    commanded: Pose,
    now: float,
    limits: CatchLimits = DEFAULT_CATCH_LIMITS,
) -> CatchStep:
    """One frame of Auto Catch. `egg` is the newest front-camera egg (duck-typed cx, h), `wrist` the
    newest wrist-view check result (anything but None = egg in view; only read in `wrist_check`),
    and `commanded` the last arm pose sent. Terminal outcomes return an idle state, zero base
    velocities, and the arm pose to hold; an idle state holds the arm and changes nothing."""
    dt = now - state.updated_at
    if state.phase == "to_start":
        return _to_start(state, commanded, now, dt, limits)
    if state.phase == "align":
        return _align(state, egg, commanded, now)
    if state.phase == "to_catch":
        return _to_catch(state, commanded, now, dt, limits)
    if state.phase == "wrist_check":
        return _wrist_check(state, wrist, commanded, now, limits)
    if state.phase == "pick_stub":  # Phase 3 placeholder for the pick policy runner: no arm motion
        return _hold(_next_phase(state, "to_release", now), commanded, "policy_stub")
    if state.phase == "to_release":
        return _to_release(state, commanded, now, dt, limits)
    return _hold(state, commanded)
