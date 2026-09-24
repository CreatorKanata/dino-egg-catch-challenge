"""src/robot/drive_loop.py: One Manual Mode control-loop iteration and the fixed-rate loop.

Polls the controller and the KachiButton commands, reads the leader arm, decides the base
action and arm pose (manual_mode.py, including a running Auto Catch or Auto Release), sends them,
applies the staff keys (save home or catch pose, record the release motion; arm_store.py), then
observes: the Pi frames are converted to BGR, the 16:9 overhead frame is downscaled once, the egg
and basket detectors run on the front frame in Manual Mode (their results drive the next frame's
`Hi!` / `Thx` checks and alignments), the wrist-view check runs only during Auto Catch's
wrist_check frames (when enabled), a `capture` command saves the frames (without overlays), and
the operator (Rerun) and attendee (signboard) views are updated.
Extracted from teleop_drive.py so the entry point only handles setup and shutdown. The loop
ends when the display reports ESC, window close, or a dead signboard process; `Stop` does not
end it. This module must not import pygame (robot.signboard); cv2 is only loaded lazily (by the
LeRobot helpers used here and inside robot.vision), so its unit tests stay free of OpenCV.
"""

from collections.abc import Mapping
from dataclasses import dataclass, replace
import logging
from pathlib import Path
import time
from typing import TYPE_CHECKING, Any, Protocol

from lerobot.utils.robot_utils import precise_sleep

from robot.config import (
    CAPTURE_COMMAND,
    FRONT_CAMERA_KEY,
    LOOP_HZ,
    SIGNBOARD_SIDE_CAMERAS,
    SPEED_LEVELS,
    TOP_CAMERA_KEY,
    WRIST_CAMERA_KEY,
)
from robot.dino_controller_reader import ControllerState, SerialControllerReader
from robot.arm_follow import ArmFollowState, disengaged
from robot.arm_store import (
    DEFAULT_PATHS,
    ArmDataPaths,
    handle_staff_keys,
    load_catch_request,
    load_release_request,
    recording_seconds,
    wants_catch,
    wants_release,
)
from robot.auto_catch import DEFAULT_CATCH_LIMITS, CatchLimits, CatchRequest
from robot.auto_release import ReleaseRequest
from robot.display_status import DisplayStatus, display_status, front_overlays
from robot.drive_state import DriveState, update_drive_state
from robot.leader_arm import LeaderArm
from robot.lekiwi_adapter import LeKiwiAdapter
from robot.align import TraceRow
from robot.display_status import ArmStatus
from robot.manual_mode import (
    LoopState,
    auto_arm_status,
    fold_commands,
    leader_wanted,
    log_changes,
    plan_arm,
    plan_base,
    step_auto_catch,
    step_auto_release,
)
from robot.mode_manager import NOTICE_CAPTURE_FAILED, NOTICE_CAPTURED, AppState, with_notice
from robot.vision.basket_detector import detect_basket
from robot.vision.basket_size import classify_basket
from robot.vision.align_trace import append_row, trace_path
from robot.vision.capture import save_capture
from robot.vision.egg_detector import detect_eggs
from robot.vision.egg_size import EggDetection, WristView, classify_size
from robot.vision.frames import normalize_observation_frames
from robot.vision.top_frame import downscale_to_width
from robot.vision.timing import record_detect_time
from robot.vision.wrist_check import egg_in_wrist_view

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
    arm_paths: ArmDataPaths = DEFAULT_PATHS  # home pose, release motion, and catch pose files
    catch_limits: CatchLimits = DEFAULT_CATCH_LIMITS  # Auto Catch arm speed, timeouts, wrist check


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


def _plan_arm(
    devices: DriveDevices, app: AppState, follow: ArmFollowState, auto_arm: dict[str, float] | None, dt: float
) -> tuple[dict[str, float], ArmFollowState, ArmStatus]:
    """The automatic action's pose when it produced one (the leader is not read); else plan_arm."""
    if auto_arm is not None:
        return auto_arm, disengaged(), auto_arm_status(app)
    has_leader = devices.leader is not None
    leader_pose = devices.leader.read_pose() if leader_wanted(app, has_leader) else None
    return plan_arm(devices.adapter.arm_hold, leader_pose, follow, app, has_leader, dt)


def _next_state(
    devices: DriveDevices, state: LoopState, now: float, dt: float
) -> tuple[LoopState, ControllerState, dict[str, float], tuple[str, ...]]:
    """Steps 1-6 of a frame: inputs and commands, Auto Catch / Auto Release, stops, leader, base,
    send, then the staff keys (they record the pose just sent).

    Returns the sent action and this frame's commands as well. The egg, basket, and wrist check used
    here are the ones from the previous frame's images (the newest available).
    """
    controller, encoder_delta = devices.reader.poll(now)
    stale = devices.reader.is_stale(now)
    commands = devices.view.poll_commands() if devices.view is not None else ()
    release = (load_release_request(devices.arm_paths, classify_basket(state.basket))
               if wants_release(state.app, commands) else ReleaseRequest())
    catch = (load_catch_request(devices.arm_paths, classify_size(state.egg))
             if wants_catch(state.app, commands) else CatchRequest())
    folded = fold_commands(state.app, commands, now, catch, release)
    arm_hold = devices.adapter.arm_hold
    folded, catch_base, catch_arm, trace = step_auto_catch(folded, state.egg, state.wrist, arm_hold, now,
                                                           devices.catch_limits)
    folded, release_base, release_arm = step_auto_release(folded, state.basket, arm_hold, now)
    follow = disengaged() if folded.disengage_arm else state.follow
    drive = update_drive_state(state.drive, controller, encoder_delta, stale, dt)
    auto_arm = catch_arm if catch_arm is not None else release_arm
    arm_cmd, follow, arm_status = _plan_arm(devices, folded.app, follow, auto_arm, dt)
    drive, base = plan_base(drive, controller, folded.app, folded.stop_base,
                            catch_base if catch_base is not None else release_base)
    devices.adapter.send_action(base, arm_cmd)
    app, recording = handle_staff_keys(folded.app, state.recording, commands, arm_cmd, arm_status, now,
                                       devices.arm_paths)
    aligning = folded.app.action == "auto_catch" and folded.app.catch.phase == "align"
    next_state = replace(state, drive=drive, app=app, follow=follow, arm_status=arm_status, recording=recording,
                         align_trace=record_trace(state.align_trace, trace, aligning))
    log_transitions(state.drive, drive)
    log_changes(state, next_state)
    return next_state, controller, {**arm_cmd, **base}, tuple(commands)


def record_trace(path: str | None, row: TraceRow | None, running: bool) -> str | None:
    """Append this frame's alignment row (a new file per alignment); log the path when it ends.

    A failed write is logged once per alignment and never stops the loop.
    """
    if row is None:
        return None
    target = str(trace_path()) if path is None else path  # "" = tracing off after a failure
    if target:
        try:
            append_row(Path(target), row)
        except OSError:
            logger.exception("Alignment trace could not be written; tracing off for this alignment")
            target = ""
    if running:
        return target
    if target:
        logger.info("Alignment trace: %s", target)
    return None


def detect_front(state: LoopState, frames: Mapping[str, Any]) -> tuple[LoopState, tuple[EggDetection, ...]]:
    """Run the egg and basket detectors on the front frame in Manual Mode; remember the best egg
    and the basket, and time the egg detector."""
    front = frames.get(FRONT_CAMERA_KEY)
    if state.app.mode != "manual" or front is None:
        return replace(state, egg=None, basket=None), ()
    started = time.perf_counter()
    detections = detect_eggs(front)
    timing = record_detect_time(state.timing, time.perf_counter() - started)
    basket = detect_basket(front)
    return replace(state, egg=detections[0] if detections else None, basket=basket, timing=timing), detections


def detect_wrist(state: LoopState, frames: Mapping[str, Any], limits: CatchLimits) -> WristView | None:
    """Run the wrist-view check only during Auto Catch's wrist_check frames and only when enabled."""
    wrist = frames.get(WRIST_CAMERA_KEY)
    if not limits.wrist_check_enabled or wrist is None or state.app.catch.phase != "wrist_check":
        return None
    return egg_in_wrist_view(wrist)


def capture(state: LoopState, frames: Mapping[str, Any], detections: tuple[EggDetection, ...], now: float) -> LoopState:
    """Save the raw frames and detections; a failed write is logged and shown, never raised."""
    try:
        path = save_capture(frames, detections, state.app.mode)
    except OSError:
        logger.exception("Capture failed")
        return replace(state, app=with_notice(state.app, NOTICE_CAPTURE_FAILED, now, level="warning"))
    logger.info("Captured %s", path)
    return replace(state, app=with_notice(state.app, NOTICE_CAPTURED, now))


def step(devices: DriveDevices, state: LoopState, previous_time: float | None) -> tuple[LoopState, bool, float]:
    """Run one iteration; return the next LoopState, whether to keep running, and its time.

    dt is 0 on the first iteration; the rotation budget and arm approach clamp it to 2 / LOOP_HZ.
    """
    now = time.monotonic()
    dt = 0.0 if previous_time is None else now - previous_time
    next_state, controller, sent, commands = _next_state(devices, state, now, dt)
    observation = normalize_observation_frames(devices.adapter.observe())
    top_raw = devices.camera.read_latest() if devices.camera is not None else None  # 1280x720, for a later tracker
    top_frame = downscale_to_width(top_raw)  # 960x540 for the signboard, Rerun, and captures
    frames = camera_frames(observation, top_frame)
    next_state, detections = detect_front(next_state, frames)
    next_state = replace(next_state, wrist=detect_wrist(next_state, frames, devices.catch_limits))
    if CAPTURE_COMMAND in commands:
        next_state = capture(next_state, frames, detections, now)
    if devices.use_rerun:
        from lerobot.utils.visualization_utils import log_rerun_data  # loads cv2; only when opted in

        logged = observation if top_frame is None else {**observation, TOP_CAMERA_KEY: top_frame}
        log_rerun_data(observation=logged, action={**sent, **drive_scalars(next_state)})
    if devices.view is None:
        return next_state, True, now
    overlays = front_overlays(next_state.app, next_state.egg, next_state.basket)
    status = display_status(next_state.app, next_state.arm_status, overlays,
                            recording_seconds(next_state.recording, now))
    devices.view.render(frames, next_state.drive, controller, status)
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
