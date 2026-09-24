"""src/robot/drive_loop.py: One Manual Mode control-loop iteration and the fixed-rate loop.

Polls the controller and the KachiButton commands, reads the leader arm, decides the base
action and arm pose (manual_mode.py), sends them, then updates the operator (Rerun) and
attendee (signboard) views. Extracted from teleop_drive.py so the entry point only handles
setup and shutdown. The loop ends when the display reports ESC, window close, or a dead
signboard process; `Stop` does not end it. This module must not import pygame
(robot.signboard); cv2-loading LeRobot modules are imported lazily or for type checking only.
"""

from collections.abc import Mapping
from dataclasses import dataclass
import logging
import time
from typing import TYPE_CHECKING, Any, Protocol

from lerobot.utils.robot_utils import precise_sleep

from robot.config import LOOP_HZ, SIGNBOARD_SIDE_CAMERAS, SPEED_LEVELS, TOP_CAMERA_KEY
from robot.dino_controller_reader import ControllerState, SerialControllerReader
from robot.arm_follow import disengaged
from robot.display_status import DisplayStatus, display_status
from robot.drive_state import DriveState, update_drive_state
from robot.leader_arm import LeaderArm
from robot.lekiwi_adapter import LeKiwiAdapter
from robot.manual_mode import LoopState, fold_commands, leader_wanted, log_changes, plan_arm, plan_base

if TYPE_CHECKING:  # top_camera loads cv2 through LeRobot; not needed at runtime here
    from robot.top_camera import TopCamera

logger = logging.getLogger(__name__)


class DisplaySink(Protocol):
    """Attendee display (SignboardClient): draw a frame, report exit, deliver commands, close."""

    def render(
        self, frames: Mapping[str, Any], drive: DriveState, controller: ControllerState, status: DisplayStatus
    ) -> None: ...

    def pump(self) -> bool: ...

    def poll_commands(self) -> tuple[str, ...]: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class DriveDevices:
    """Everything the loop talks to; optional parts are None when disabled on the CLI."""

    reader: SerialControllerReader
    adapter: LeKiwiAdapter
    camera: "TopCamera | None"
    view: DisplaySink | None
    use_rerun: bool
    leader: LeaderArm | None = None


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
    drive = state.drive
    return {
        "drive.speed_index": float(drive.speed_index),
        "drive.catch_requested": float(drive.catch_requested),
        "drive.input_lost": float(drive.input_lost),
        "drive.pending_rotation_deg": float(drive.pending_rotation_deg),
        "mode.fsc": float(state.app.mode == "fsc"),
        "arm.engaged": float(state.follow.engaged),
    }


def camera_frames(observation: dict[str, Any], top_frame: Any | None) -> dict[str, Any | None]:
    """Frames for the signboard in display order: overhead, then the Pi cameras."""
    return {TOP_CAMERA_KEY: top_frame, **{name: observation.get(name) for name in SIGNBOARD_SIDE_CAMERAS}}


def _next_state(
    devices: DriveDevices, state: LoopState, now: float, dt: float
) -> tuple[LoopState, ControllerState, dict[str, float]]:
    """Steps 1-5 of a frame: inputs and commands, stops, leader, base, then send (returned too)."""
    controller, encoder_delta = devices.reader.poll(now)
    stale = devices.reader.is_stale(now)
    commands = devices.view.poll_commands() if devices.view is not None else ()
    folded = fold_commands(state.app, commands, now)
    follow = disengaged() if folded.disengage_arm else state.follow
    drive = update_drive_state(state.drive, controller, encoder_delta, stale, dt)
    has_leader = devices.leader is not None
    leader_pose = devices.leader.read_pose() if leader_wanted(folded.app, has_leader) else None
    arm_cmd, follow, arm_status = plan_arm(devices.adapter.arm_hold, leader_pose, follow, folded.app, has_leader, dt)
    drive, base = plan_base(drive, controller, folded.app, folded.stop_base)
    devices.adapter.send_action(base, arm_cmd)
    next_state = LoopState(drive=drive, app=folded.app, follow=follow, arm_status=arm_status)
    log_transitions(state.drive, drive)
    log_changes(state, next_state)
    return next_state, controller, {**arm_cmd, **base}


def step(devices: DriveDevices, state: LoopState, previous_time: float | None) -> tuple[LoopState, bool, float]:
    """Run one iteration; return the next LoopState, whether to keep running, and its time.

    dt is 0 on the first iteration; the rotation budget and arm approach clamp it to 2 / LOOP_HZ.
    """
    now = time.monotonic()
    dt = 0.0 if previous_time is None else now - previous_time
    next_state, controller, sent = _next_state(devices, state, now, dt)
    observation = devices.adapter.observe()
    top_frame = devices.camera.read_latest() if devices.camera is not None else None
    if devices.use_rerun:
        from lerobot.utils.visualization_utils import log_rerun_data  # loads cv2; only when opted in

        logged = observation if top_frame is None else {**observation, TOP_CAMERA_KEY: top_frame}
        log_rerun_data(observation=logged, action={**sent, **drive_scalars(next_state)})
    if devices.view is None:
        return next_state, True, now
    status = display_status(next_state.app, next_state.arm_status)
    devices.view.render(camera_frames(observation, top_frame), next_state.drive, controller, status)
    return next_state, devices.view.pump(), now


def loop(devices: DriveDevices) -> None:
    """Run at LOOP_HZ until the display ends; exceptions propagate to the caller."""
    period = 1.0 / LOOP_HZ
    state = LoopState()
    keep_running = True
    previous_time: float | None = None
    while keep_running:
        started = time.perf_counter()
        state, keep_running, previous_time = step(devices, state, previous_time)
        precise_sleep(max(period - (time.perf_counter() - started), 0.0))
