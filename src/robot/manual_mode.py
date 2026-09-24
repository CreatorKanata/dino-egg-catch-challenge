"""src/robot/manual_mode.py: Per-frame Manual Mode decisions for the control loop.

Folds KachiButton commands through the mode manager (`Hi!` in Manual Mode with the latest egg
size class and the recorded poses, `Thx` with the basket size class and the recorded data),
advances a running Auto Catch or Auto Release, then decides the base action (the action's command
while it runs, else controller driving only in Manual Mode, never in a stop frame or while the Stop
latch holds) and the arm pose (the action's pose while it runs; else slow engagement, then leader
following; held in FSC, while stopped, and on leader faults), and whether the front-camera detectors
run this frame (not in action phases that ignore them). Also the loop's change logs and its
Rerun scalars. Split out of drive_loop.py to keep files small. Stdlib-only: no hardware is touched
here, so every rule is unit-tested.
"""

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field, replace
import logging
from typing import Any, Final

from robot.align import TraceRow, trace_row
from robot.arm_follow import ArmFollowState, disengaged, follow_step
from robot.arm_store import RecordingState
from robot.auto_catch import DEFAULT_CATCH_LIMITS, CatchLimits, CatchRequest, PolicyAct, catch_step
from robot.auto_release import ReleaseRequest, release_step
from robot.config import LEFT_RIGHT_ROLE, SPEED_LEVELS
from robot.controller_to_action import stop_action
from robot.dino_controller_reader import ControllerState
from robot.display_status import ArmStatus
from robot.drive_state import DriveState, select_action
from robot.mode_manager import (
    AppState,
    Transition,
    apply_command,
    catch_notice,
    expire_notice,
    manual_control_allowed,
    release_notice,
    start_auto_catch,
    start_auto_release,
)
from robot.vision.basket_size import BasketDetection
from robot.vision.egg_size import EggDetection, WristView
from robot.vision.timing import DetectTiming

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class LoopState:
    """Everything the loop carries from one frame to the next."""

    drive: DriveState = DriveState()
    app: AppState = AppState()
    follow: ArmFollowState = ArmFollowState()
    arm_status: ArmStatus = "holding"
    egg: EggDetection | None = None  # best egg in the latest front frame (Manual Mode only)
    timing: DetectTiming = DetectTiming()
    align_trace: str | None = None  # CSV trace path of the running alignment
    basket: BasketDetection | None = None  # pink basket in the latest front frame (Manual Mode only)
    recording: RecordingState = RecordingState()  # release motion recording (staff key)
    wrist: WristView | None = None  # wrist-view check of the latest frame (only during Auto Catch's check)
    # The latest raw observation (Pi frames RGB-ordered, as LeKiwiClient delivers them) for the pick policy.
    observation: Mapping[str, Any] | None = field(default=None, compare=False, repr=False)


@dataclass(frozen=True)
class CommandResult:
    """Mode state after this frame's commands, and whether any of them asked for a stop."""

    app: AppState
    stop_base: bool
    disengage_arm: bool


def _apply(app: AppState, command: str, now: float, catch: CatchRequest, release: ReleaseRequest,
           limits: CatchLimits) -> Transition:
    if command == "hi" and app.mode == "manual":
        return start_auto_catch(app, catch, now, limits=limits)
    if command == "thx" and app.mode == "manual":
        return start_auto_release(app, release, now)
    return apply_command(app, command, now)


def fold_commands(
    app: AppState,
    commands: Iterable[str],
    now: float,
    catch: CatchRequest = CatchRequest(),
    release: ReleaseRequest = ReleaseRequest(),
    catch_limits: CatchLimits = DEFAULT_CATCH_LIMITS,
) -> CommandResult:
    """Apply commands in arrival order, then expire the notice; stop flags are OR-ed.

    `hi` in Manual Mode goes to start_auto_catch with `catch` (the latest front frame's size class
    and the loaded poses), `thx` in Manual Mode to start_auto_release with `release` (basket size
    class and the loaded data); every other command goes to apply_command. Unknown commands
    ("capture", "save_home", "save_catch", "toggle_record") are no-ops here.
    """
    stop_base = disengage_arm = False
    for command in commands:
        transition = _apply(app, command, now, catch, release, catch_limits)
        app = transition.state
        stop_base = stop_base or transition.stop_base
        disengage_arm = disengage_arm or transition.disengage_arm
    return CommandResult(app=expire_notice(app, now), stop_base=stop_base, disengage_arm=disengage_arm)


def step_auto_catch(
    result: CommandResult, egg: EggDetection | None, wrist: WristView | None, commanded: Mapping[str, float],
    now: float, limits: CatchLimits = DEFAULT_CATCH_LIMITS, observation: Mapping[str, Any] | None = None,
    policy_act: PolicyAct | None = None,
) -> tuple[CommandResult, dict[str, float] | None, dict[str, float] | None, TraceRow | None]:
    """Advance a running Auto Catch on this frame's egg, wrist check, last commanded arm pose, and
    (in `pick`) the raw observation and the pick policy runner (None = stub).

    Returns (result, base action, arm pose, alignment trace row), or (result, None, None, None) when
    it is not running. Base and arm replace controller driving and leader following for this frame;
    the trace row exists only for alignment frames. A terminal outcome ends the action
    (catch_notice): zero velocities this frame and arm following disengaged.
    """
    app = result.app
    if app.action != "auto_catch":
        return result, None, None, None
    step = catch_step(app.catch, egg, wrist, commanded, now, limits, observation, policy_act)
    row = None if step.align_result is None else trace_row(app.catch.align, egg, now, step.base, step.align_result)
    transition = catch_notice(replace(app, catch=step.state), step.outcome, now)
    return CommandResult(app=transition.state, stop_base=result.stop_base or transition.stop_base,
                         disengage_arm=result.disengage_arm or transition.disengage_arm), step.base, step.arm, row


def step_auto_release(
    result: CommandResult, basket: BasketDetection | None, commanded: Mapping[str, float], now: float
) -> tuple[CommandResult, dict[str, float] | None, dict[str, float] | None]:
    """Advance a running Auto Release on this frame's basket and the last commanded arm pose.

    Returns (result, base action, arm pose), or (result, None, None) when it is not running. Both
    replace controller driving and leader following for this frame. A terminal outcome ends the
    action (release_notice): zero velocities this frame and arm following disengaged.
    """
    app = result.app
    if app.action != "auto_release":
        return result, None, None
    step = release_step(app.release, basket, commanded, now)
    transition = release_notice(replace(app, release=step.state), step.outcome, now)
    return CommandResult(app=transition.state, stop_base=result.stop_base or transition.stop_base,
                         disengage_arm=result.disengage_arm or transition.disengage_arm), step.base, step.arm


def auto_arm_status(app: AppState) -> ArmStatus:
    """Arm status for a frame whose pose came from an automatic action (holding on its last frame)."""
    if app.action == "auto_catch":
        return "auto catch"
    return "auto release" if app.action == "auto_release" else "holding"


# Automatic-action phases that do not read the front camera: its egg and basket detection is skipped
# there to keep the 30 Hz frame budget (the pick policy's inference runs in `pick`).
NO_DETECT_PHASES: Final = {"auto_catch": ("to_start", "to_catch", "pick", "to_release"),
                           "auto_release": ("home", "play", "return_home")}


def front_detection_wanted(app: AppState) -> bool:
    """Run the egg and basket detectors: Manual Mode, except in the action phases listed above."""
    if app.mode != "manual":
        return False
    phase = app.catch.phase if app.action == "auto_catch" else app.release.phase
    return phase not in NO_DETECT_PHASES.get(app.action, ())


def leader_wanted(app: AppState, has_leader: bool) -> bool:
    """Read the leader only when it can drive the arm: a leader exists, Manual Mode, not
    stopped, and no automatic action running."""
    return has_leader and manual_control_allowed(app)


def plan_base(
    drive: DriveState,
    controller: ControllerState,
    app: AppState,
    stopping: bool,
    auto_base: dict[str, float] | None = None,
) -> tuple[DriveState, dict[str, float]]:
    """A stop frame always sends zeros. Otherwise the automatic action's command when there is
    one (controller input, including its loss, is ignored), else controller driving in Manual
    Mode. Zeros and no pending rotation in FSC, while stopped, or with no usable input."""
    if stopping:
        return replace(drive, pending_rotation_deg=0.0), stop_action()
    if auto_base is not None:
        return replace(drive, pending_rotation_deg=0.0), dict(auto_base)
    if manual_control_allowed(app):
        return drive, select_action(drive, controller, SPEED_LEVELS, LEFT_RIGHT_ROLE)
    return replace(drive, pending_rotation_deg=0.0), stop_action()


def plan_arm(
    commanded: Mapping[str, float],
    leader_pose: Mapping[str, float] | None,
    follow: ArmFollowState,
    app: AppState,
    has_leader: bool,
    dt: float,
) -> tuple[dict[str, float], ArmFollowState, ArmStatus]:
    """Arm pose to send, next follow state, and the status shown on the signboard.

    A leader fault also disengages following, so a recovered leader is approached slowly again
    instead of jumping to wherever it was moved during the fault.
    """
    if not has_leader:
        return dict(commanded), disengaged(), "no leader"
    if not leader_wanted(app, has_leader):
        return dict(commanded), disengaged(), "holding"
    if leader_pose is None:
        return dict(commanded), disengaged(), "leader fault"
    pose, next_follow = follow_step(commanded, leader_pose, follow, dt)
    return pose, next_follow, "following" if next_follow.engaged else "syncing"


def log_changes(before: LoopState, after: LoopState) -> None:
    """Log mode, action, notice, and arm status changes once per change."""
    if after.app.stopped != before.app.stopped:
        if after.app.stopped:
            logger.warning("STOP: base zeroed, arm held; press Go Go! to resume")
        else:
            logger.info("Stop released by Go Go!")
    if after.app.mode != before.app.mode:
        logger.info("Mode: %s", after.app.mode)
    if after.app.action != before.app.action:
        logger.info("Action: %s", after.app.action)
    if after.app.notice != before.app.notice and after.app.notice:
        logger.info("Notice: %s", after.app.notice)
    if after.app.voice_listening != before.app.voice_listening:
        logger.info("Voice input %s", "started" if after.app.voice_listening else "ended")
    if after.arm_status != before.arm_status:
        logger.info("Arm: %s", after.arm_status)


def log_transitions(before: DriveState, after: DriveState) -> None:
    """Log input loss, Catch requests, and speed changes once per change, not every frame."""
    if after.input_lost != before.input_lost:
        if after.input_lost:
            logger.warning("Controller input lost; base stopped")
        else:
            logger.info("Controller input synchronized")
    if after.catch_requested != before.catch_requested:
        logger.info("Catch %s", "requested; base stopped" if after.catch_requested else "released")
    if after.speed_index != before.speed_index:
        logger.info("Speed level %d of %d", after.speed_index + 1, len(SPEED_LEVELS))


def drive_scalars(state: LoopState) -> dict[str, float]:
    """Drive and mode values logged to Rerun next to the action."""
    drive = state.drive
    return {
        "drive.speed_index": float(drive.speed_index),
        "drive.catch_requested": float(drive.catch_requested),
        "drive.input_lost": float(drive.input_lost),
        "drive.pending_rotation_deg": float(drive.pending_rotation_deg),
        "mode.fsc": float(state.app.mode == "fsc"),
        "arm.engaged": float(state.follow.engaged),
    }
