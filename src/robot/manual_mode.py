"""src/robot/manual_mode.py: Per-frame Manual Mode decisions for the control loop.

Folds KachiButton commands through the mode manager (`Hi!` in Manual Mode with the latest egg
size class, `Thx` with the basket size class and the recorded data), advances a running Auto Catch
alignment or Auto Release, then decides the base action (the action's command while it runs, else
controller driving only in Manual Mode, never in a stop frame or while the Stop latch holds) and
the arm pose (Auto Release's pose while it runs; else slow engagement, then leader following;
held in FSC, while stopped, during the alignment, and on leader faults). Split out of
drive_loop.py to keep files small. Stdlib-only: no hardware is touched here, so every rule is
unit-tested.
"""

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, replace
import logging

from robot.align import TraceRow, align_step, trace_row
from robot.arm_follow import ArmFollowState, disengaged, follow_step
from robot.arm_store import RecordingState
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
    expire_notice,
    finish_auto_catch,
    manual_control_allowed,
    release_notice,
    start_auto_catch,
    start_auto_release,
)
from robot.vision.basket_size import BasketDetection
from robot.vision.egg_size import EggDetection, SizeClass
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


@dataclass(frozen=True)
class CommandResult:
    """Mode state after this frame's commands, and whether any of them asked for a stop."""

    app: AppState
    stop_base: bool
    disengage_arm: bool


def _apply(app: AppState, command: str, now: float, egg_size: SizeClass, release: ReleaseRequest) -> Transition:
    if command == "hi" and app.mode == "manual":
        return start_auto_catch(app, egg_size, now)
    if command == "thx" and app.mode == "manual":
        return start_auto_release(app, release, now)
    return apply_command(app, command, now)


def fold_commands(
    app: AppState,
    commands: Iterable[str],
    now: float,
    egg_size: SizeClass = "none",
    release: ReleaseRequest = ReleaseRequest(),
) -> CommandResult:
    """Apply commands in arrival order, then expire the notice; stop flags are OR-ed.

    `hi` in Manual Mode goes to start_auto_catch with `egg_size` (the latest front frame's size
    class), `thx` in Manual Mode to start_auto_release with `release` (basket size class and the
    loaded data); every other command goes to apply_command. Unknown commands ("capture",
    "save_home", "toggle_record") are no-ops here.
    """
    stop_base = disengage_arm = False
    for command in commands:
        transition = _apply(app, command, now, egg_size, release)
        app = transition.state
        stop_base = stop_base or transition.stop_base
        disengage_arm = disengage_arm or transition.disengage_arm
    return CommandResult(app=expire_notice(app, now), stop_base=stop_base, disengage_arm=disengage_arm)


def step_auto_catch(
    result: CommandResult, egg: EggDetection | None, now: float
) -> tuple[CommandResult, dict[str, float] | None, TraceRow | None]:
    """Advance a running alignment on this frame's egg; (result, None, None) when no action runs.

    The returned base action replaces controller driving; the trace row describes the frame. A
    terminal alignment result ends the action (finish_auto_catch): zero velocities this frame and
    arm following disengaged.
    """
    app = result.app
    if app.action != "auto_catch":
        return result, None, None
    align, base, outcome = align_step(app.align, egg, now)
    row = trace_row(app.align, egg, now, base, outcome)
    if outcome == "running":
        return replace(result, app=replace(app, align=align)), base, row
    finished = finish_auto_catch(app, outcome, now)
    return CommandResult(app=finished.state, stop_base=result.stop_base or finished.stop_base,
                         disengage_arm=True), base, row


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


def release_arm_status(app: AppState) -> ArmStatus:
    """Arm status for a frame whose pose came from Auto Release (holding on its last frame)."""
    return "auto release" if app.action == "auto_release" else "holding"


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
