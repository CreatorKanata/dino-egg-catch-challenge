"""src/robot/drive_state.py: Immutable Manual Mode driving state and the per-frame action decision.

Combines controller input, encoder steps, loop timing, and input health into one frozen
DriveState and chooses the base action. Encoder clicks become a rotation budget in degrees
that is spent at the speed level's theta speed. Stdlib-only so the stop rules (input lost,
Catch request) and the rotation budget are verified by unit tests without hardware.
"""

from collections.abc import Sequence
from dataclasses import dataclass
import math

from robot.config import (
    ENCODER_DEGREES_PER_STEP,
    ENCODER_MAX_PENDING_DEG,
    ENCODER_ROLE,
    INITIAL_SPEED_INDEX,
    LOOP_HZ,
    ROLE_CATCH,
    ROLE_ROTATE_BASE,
    ROLE_SPEED,
    SHAFT_BUTTON_ROLE,
    SPEED_LEVELS,
    SpeedLevel,
)
from robot.controller_to_action import base_action, step_speed_index, stop_action
from robot.dino_controller_reader import ControllerState

MAX_FRAME_DT_S = 2 / LOOP_HZ  # a stalled frame must not consume a large chunk of budget at once


@dataclass(frozen=True)
class DriveState:
    """Speed level, Catch request, input health, and the encoder rotation still to perform.

    `pending_rotation_deg` > 0 means rotate left (+theta), < 0 means rotate right (-theta).
    """

    speed_index: int = INITIAL_SPEED_INDEX
    catch_requested: bool = False
    input_lost: bool = True
    pending_rotation_deg: float = 0.0


def consume_rotation(pending: float, theta_speed: float, dt: float) -> float:
    """Move `pending` toward zero by what `theta_speed` covers in `dt` (clamped); no overshoot.

    Open-loop: this is the rotation that was commanded during the last frame, not a measurement.
    """
    frame_dt = min(max(dt, 0.0), MAX_FRAME_DT_S)
    spent = min(abs(pending), theta_speed * frame_dt)
    return pending - math.copysign(spent, pending)


def _add_steps(pending: float, encoder_delta: int, degrees_per_step: float, max_pending: float) -> float:
    """cw (delta +1) rotates right, so it lowers the budget, matching LeKiwi rotate_right = -theta."""
    target = pending - encoder_delta * degrees_per_step
    return max(-max_pending, min(max_pending, target))


def update_drive_state(
    drive: DriveState,
    controller: ControllerState,
    encoder_delta: int,
    stale: bool,
    dt: float,
    *,
    encoder_role: str = ENCODER_ROLE,
    shaft_button_role: str = SHAFT_BUTTON_ROLE,
    speed_levels: Sequence[SpeedLevel] = SPEED_LEVELS,
    degrees_per_step: float = ENCODER_DEGREES_PER_STEP,
    max_pending_deg: float = ENCODER_MAX_PENDING_DEG,
) -> DriveState:
    """Return the next DriveState; never mutates its inputs.

    Order per frame: spend the budget for the time since the previous frame, then add new steps.
    A stop (input lost or Catch) clears the budget so rotation never resumes on its own.
    """
    speed_index = drive.speed_index
    if encoder_role == ROLE_SPEED:
        speed_index = step_speed_index(speed_index, encoder_delta, len(speed_levels))
    pending = consume_rotation(drive.pending_rotation_deg, speed_levels[speed_index].theta, dt)
    if encoder_role == ROLE_ROTATE_BASE:
        pending = _add_steps(pending, encoder_delta, degrees_per_step, max_pending_deg)
    catch_requested = shaft_button_role == ROLE_CATCH and controller.button
    input_lost = stale or not controller.synchronized
    return DriveState(
        speed_index=speed_index,
        catch_requested=catch_requested,
        input_lost=input_lost,
        pending_rotation_deg=0.0 if (input_lost or catch_requested) else pending,
    )


def select_action(
    drive: DriveState,
    controller: ControllerState,
    speed_levels: Sequence[SpeedLevel],
    left_right_role: str,
) -> dict[str, float]:
    """Stop when input is lost or Catch is requested; otherwise joystick translation plus rotation."""
    if drive.input_lost or drive.catch_requested:
        return stop_action()
    speed = speed_levels[drive.speed_index]
    pending = drive.pending_rotation_deg
    theta_vel = math.copysign(speed.theta, pending) if pending else None
    return base_action(controller, speed, left_right_role, theta_vel)
