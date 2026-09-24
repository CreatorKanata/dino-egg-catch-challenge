"""src/robot/drive_loop.py: One Drive Mode control-loop iteration and the fixed-rate loop.

Polls the controller, decides the base action, sends it, then updates the operator (Rerun)
and attendee (signboard) views. Extracted from teleop_drive.py so the entry point only
handles setup and shutdown. The loop ends when the display reports ESC, window close, or a
dead signboard process. This module must not import pygame (robot.signboard); cv2-loading
LeRobot modules are imported lazily or for type checking only, keeping it light for tests.
"""

from collections.abc import Mapping
from dataclasses import dataclass
import logging
import time
from typing import TYPE_CHECKING, Any, Protocol

from lerobot.utils.robot_utils import precise_sleep

from robot.config import LEFT_RIGHT_ROLE, LOOP_HZ, SIGNBOARD_SIDE_CAMERAS, SPEED_LEVELS, TOP_CAMERA_KEY
from robot.dino_controller_reader import ControllerState, SerialControllerReader
from robot.drive_state import DriveState, select_action, update_drive_state
from robot.lekiwi_adapter import LeKiwiAdapter

if TYPE_CHECKING:  # top_camera loads cv2 through LeRobot; not needed at runtime here
    from robot.top_camera import TopCamera

logger = logging.getLogger(__name__)


class DisplaySink(Protocol):
    """Attendee display (SignboardClient in production): draw a frame, report exit, close."""

    def render(self, frames: Mapping[str, Any], drive: DriveState, controller: ControllerState) -> None: ...

    def pump(self) -> bool: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class DriveDevices:
    """Everything the loop talks to; optional parts are None when disabled on the CLI."""

    reader: SerialControllerReader
    adapter: LeKiwiAdapter
    camera: "TopCamera | None"
    view: DisplaySink | None
    use_rerun: bool


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


def drive_scalars(drive: DriveState) -> dict[str, float]:
    return {
        "drive.speed_index": float(drive.speed_index),
        "drive.catch_requested": float(drive.catch_requested),
        "drive.input_lost": float(drive.input_lost),
        "drive.pending_rotation_deg": float(drive.pending_rotation_deg),
    }


def camera_frames(observation: dict[str, Any], top_frame: Any | None) -> dict[str, Any | None]:
    """Frames for the signboard in display order: overhead, then the Pi cameras."""
    return {TOP_CAMERA_KEY: top_frame, **{name: observation.get(name) for name in SIGNBOARD_SIDE_CAMERAS}}


def step(
    devices: DriveDevices, drive: DriveState, previous_time: float | None
) -> tuple[DriveState, bool, float]:
    """Run one iteration; return the next DriveState, whether to keep running, and its time.

    dt is 0 on the first iteration; update_drive_state clamps it to [0, 2 / LOOP_HZ].
    """
    now = time.monotonic()
    dt = 0.0 if previous_time is None else now - previous_time
    controller, encoder_delta = devices.reader.poll(now)
    stale = devices.reader.is_stale(now)
    next_drive = update_drive_state(drive, controller, encoder_delta, stale, dt)
    action = select_action(next_drive, controller, SPEED_LEVELS, LEFT_RIGHT_ROLE)
    devices.adapter.send_action(action)
    log_transitions(drive, next_drive)
    observation = devices.adapter.observe()
    top_frame = devices.camera.read_latest() if devices.camera is not None else None
    if devices.use_rerun:
        from lerobot.utils.visualization_utils import log_rerun_data  # loads cv2; only when opted in

        logged = observation if top_frame is None else {**observation, TOP_CAMERA_KEY: top_frame}
        log_rerun_data(observation=logged, action={**action, **drive_scalars(next_drive)})
    if devices.view is None:
        return next_drive, True, now
    devices.view.render(camera_frames(observation, top_frame), next_drive, controller)
    return next_drive, devices.view.pump(), now


def loop(devices: DriveDevices) -> None:
    """Run at LOOP_HZ until the display ends; exceptions propagate to the caller."""
    period = 1.0 / LOOP_HZ
    drive = DriveState()
    keep_running = True
    previous_time: float | None = None
    while keep_running:
        started = time.perf_counter()
        drive, keep_running, previous_time = step(devices, drive, previous_time)
        precise_sleep(max(period - (time.perf_counter() - started), 0.0))
