"""src/robot/manual_mode.py: Per-frame Manual Mode decisions for the control loop (Phase 1).

Folds KachiButton commands through the mode manager, then decides the base action (controller
driving only in Manual Mode, never in a stop frame or while the Stop latch holds) and the arm
pose (slow engagement, then leader following; held in FSC, while stopped, and on leader
faults). Split out of drive_loop.py to keep files small. Stdlib-only: no hardware is touched
here, so every rule is unit-tested.
"""

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, replace
import logging

from robot.arm_follow import ArmFollowState, disengaged, follow_step
from robot.config import LEFT_RIGHT_ROLE, SPEED_LEVELS
from robot.controller_to_action import stop_action
from robot.dino_controller_reader import ControllerState
from robot.display_status import ArmStatus
from robot.drive_state import DriveState, select_action
from robot.mode_manager import AppState, apply_command, expire_notice, manual_control_allowed

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class LoopState:
    """Everything the loop carries from one frame to the next."""

    drive: DriveState = DriveState()
    app: AppState = AppState()
    follow: ArmFollowState = ArmFollowState()
    arm_status: ArmStatus = "holding"


@dataclass(frozen=True)
class CommandResult:
    """Mode state after this frame's commands, and whether any of them asked for a stop."""

    app: AppState
    stop_base: bool
    disengage_arm: bool


def fold_commands(app: AppState, commands: Iterable[str], now: float) -> CommandResult:
    """Apply commands in arrival order, then expire the notice; stop flags are OR-ed."""
    stop_base = disengage_arm = False
    for command in commands:
        transition = apply_command(app, command, now)
        app = transition.state
        stop_base = stop_base or transition.stop_base
        disengage_arm = disengage_arm or transition.disengage_arm
    return CommandResult(app=expire_notice(app, now), stop_base=stop_base, disengage_arm=disengage_arm)


def leader_wanted(app: AppState, has_leader: bool) -> bool:
    """Read the leader only when it can drive the arm: a leader exists, Manual Mode, not stopped."""
    return has_leader and manual_control_allowed(app)


def plan_base(
    drive: DriveState, controller: ControllerState, app: AppState, stopping: bool
) -> tuple[DriveState, dict[str, float]]:
    """Controller driving in Manual Mode; zeros and no pending rotation in FSC, a stop frame, or
    while the Stop latch holds (manual_control_allowed is False then)."""
    if manual_control_allowed(app) and not stopping:
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
