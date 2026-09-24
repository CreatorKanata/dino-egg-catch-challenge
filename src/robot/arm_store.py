"""src/robot/arm_store.py: Staff keys that record the arm data, and loading it for `Thx` and `Hi!`.

Staff record the release pose (home), the catch pose, and the release motion from the signboard
keyboard in Manual Mode with the leader arm following (src/robot/README.md, "Auto Release" and
"Auto Catch"). `b` (HOME_KEY) saves the current commanded arm pose as the home pose; `k`
(CATCH_POSE_KEY) saves it as the catch pose, only in Manual Mode with the arm following and no
action running. `r` (RECORD_KEY) starts recording the commanded pose every
loop frame and, pressed again, writes the release motion (the previous file is kept as
`release_motion.prev.json`). Recording is refused outside Manual Mode or when the arm is not
following the leader, is cancelled (nothing written) if following stops meanwhile, and stops and
saves itself after RECORD_MAX_S. At a `Thx` press the loop loads the home pose and the motion into a
ReleaseRequest, at a `Hi!` press the catch and home poses into a CatchRequest; a missing or invalid
file becomes None, so the action refuses with a warning and nothing moves. Stdlib-only; the file
writes are the only side effects, and every failure is logged and shown, never raised.
"""

from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass, replace
from datetime import datetime
import logging
from pathlib import Path
from typing import Final

from robot.arm_motions import (
    ArmFileError,
    MotionFrame,
    home_record,
    load_motion,
    load_pose,
    motion_record,
    playback_frames,
    write_record,
)
from robot.auto_catch import CatchRequest
from robot.auto_release import ReleaseRequest
from robot.config import (
    CATCH_POSE_PATH,
    HOME_POSE_PATH,
    LOOP_HZ,
    RECORD_MAX_S,
    RELEASE_MOTION_PATH,
    REPO_ROOT,
    SAVE_CATCH_COMMAND,
    SAVE_HOME_COMMAND,
    TOGGLE_RECORD_COMMAND,
)
from robot.mode_manager import AppState, manual_control_allowed, with_notice
from robot.vision.basket_size import BasketSizeClass
from robot.vision.egg_size import SizeClass

logger = logging.getLogger(__name__)

NOTICE_HOME_SAVED: Final = "Home pose saved"
NOTICE_HOME_FAILED: Final = "Home pose not saved"
NOTICE_CATCH_SAVED: Final = "Catch pose saved"
NOTICE_CATCH_FAILED: Final = "Catch pose not saved"
NOTICE_RECORDING: Final = "Recording release..."
NOTICE_MOTION_SAVED: Final = "Release motion saved ({frames} frames, {seconds:.1f} s)"
NOTICE_MOTION_SAVED_MAX: Final = "Release motion saved (max length)"
NOTICE_MOTION_FAILED: Final = "Release motion not saved"
NOTICE_TOO_SHORT: Final = "Recording too short"
NOTICE_NOT_MANUAL: Final = "Cannot record: not in Manual Mode"
NOTICE_NOT_FOLLOWING: Final = "Cannot record: arm not following"
NOTICE_CANCELLED: Final = "Recording cancelled"
HOME_NOTE: Final = "Release pose (home): neck folded, head up. Commanded arm pose saved with the HOME_KEY."
CATCH_NOTE: Final = "Catch pose: head down, the wrist camera sees the aligned egg. Saved with the CATCH_POSE_KEY."
MOTION_NOTE: Final = ("Release motion recorded from the leader arm with the RECORD_KEY: commanded arm poses at the "
                      "loop rate, t in seconds since the recording started.")


@dataclass(frozen=True)
class ArmDataPaths:
    """Where the home pose, the release motion, and the catch pose live (the tests pass temporary paths)."""

    home: Path = REPO_ROOT / HOME_POSE_PATH
    motion: Path = REPO_ROOT / RELEASE_MOTION_PATH
    catch: Path = REPO_ROOT / CATCH_POSE_PATH


@dataclass(frozen=True)
class RecordingState:
    """Whether the release motion is being recorded, since when, and the frames so far."""

    active: bool = False
    started_at: float = 0.0
    frames: tuple[MotionFrame, ...] = ()


DEFAULT_PATHS: Final = ArmDataPaths()


def recording_seconds(recording: RecordingState, now: float) -> float | None:
    """Seconds recorded so far, or None when not recording."""
    return max(0.0, now - recording.started_at) if recording.active else None


def _timestamp() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def _load(loader: Callable[[Path], object], path: Path, what: str) -> object | None:
    try:
        return loader(path)
    except FileNotFoundError:
        logger.warning("No %s at %s; record it first with its staff key", what, path)
    except (OSError, ArmFileError) as error:
        logger.error("Invalid %s at %s: %s", what, path, error)
    return None


def load_release_request(paths: ArmDataPaths, size: BasketSizeClass) -> ReleaseRequest:
    """The Thx check's inputs: basket size class, home pose, and playback frames (None if missing)."""
    home = _load(load_pose, paths.home, "home pose")
    motion = _load(load_motion, paths.motion, "release motion")
    frames = playback_frames(motion) if motion is not None else None
    return ReleaseRequest(size=size, home=home, frames=frames)


def wants_release(app: AppState, commands: Iterable[str]) -> bool:
    """Load the data only for a Thx press that can start Auto Release."""
    return "thx" in commands and manual_control_allowed(app)


def load_catch_request(
    paths: ArmDataPaths, size: SizeClass, arm: Mapping[str, float] | None = None
) -> CatchRequest:
    """The Hi! check's inputs: egg size class, catch pose, and release pose (home; None if missing),
    and the arm pose commanded now."""
    return CatchRequest(size=size, catch=_load(load_pose, paths.catch, "catch pose"),
                        home=_load(load_pose, paths.home, "home pose"), arm=None if arm is None else dict(arm))


def wants_catch(app: AppState, commands: Iterable[str]) -> bool:
    """Load the poses only for a Hi! press that can start Auto Catch."""
    return "hi" in commands and manual_control_allowed(app)


def _save_home(app: AppState, commanded: Mapping[str, float], now: float, paths: ArmDataPaths) -> AppState:
    if app.action != "none":  # a press while an automatic action runs is ignored
        return app
    try:
        write_record(paths.home, home_record(commanded, _timestamp(), HOME_NOTE))
    except (OSError, ArmFileError):
        logger.exception("Home pose could not be written to %s", paths.home)
        return with_notice(app, NOTICE_HOME_FAILED, now, level="warning")
    logger.info("Home pose saved to %s: %s", paths.home, dict(commanded))
    return with_notice(app, NOTICE_HOME_SAVED, now)


def _save_catch(app: AppState, commanded: Mapping[str, float], arm_status: str, now: float,
                paths: ArmDataPaths) -> AppState:
    """Save the catch pose: only in Manual Mode with the arm following and no action running."""
    refusal = _can_record(app, arm_status)
    if refusal is not None:
        return with_notice(app, refusal, now, level="warning")
    try:
        write_record(paths.catch, home_record(commanded, _timestamp(), CATCH_NOTE))
    except (OSError, ArmFileError):
        logger.exception("Catch pose could not be written to %s", paths.catch)
        return with_notice(app, NOTICE_CATCH_FAILED, now, level="warning")
    logger.info("Catch pose saved to %s: %s", paths.catch, dict(commanded))
    return with_notice(app, NOTICE_CATCH_SAVED, now)


def _can_record(app: AppState, arm_status: str) -> str | None:
    """None when recording is possible, else the refusal notice."""
    if app.mode != "manual":
        return NOTICE_NOT_MANUAL
    if app.stopped or app.action != "none" or arm_status != "following":
        return NOTICE_NOT_FOLLOWING
    return None


def _finish(
    app: AppState, recording: RecordingState, now: float, paths: ArmDataPaths, saved_notice: str | None = None
) -> AppState:
    frames = recording.frames
    try:
        record = motion_record(frames, LOOP_HZ, _timestamp(), MOTION_NOTE)
    except ArmFileError:
        logger.warning("Release recording too short (%d frames); nothing written", len(frames))
        return with_notice(app, NOTICE_TOO_SHORT, now, level="warning")
    try:
        write_record(paths.motion, record, backup=True)
    except OSError:
        logger.exception("Release motion could not be written to %s", paths.motion)
        return with_notice(app, NOTICE_MOTION_FAILED, now, level="warning")
    seconds = frames[-1].t - frames[0].t
    logger.info("Release motion saved to %s (%d frames, %.1f s)", paths.motion, len(frames), seconds)
    notice = saved_notice or NOTICE_MOTION_SAVED.format(frames=len(frames), seconds=seconds)
    return with_notice(app, notice, now)


def _toggle(
    app: AppState, recording: RecordingState, arm_status: str, now: float, paths: ArmDataPaths
) -> tuple[AppState, RecordingState]:
    if recording.active:
        return _finish(app, recording, now, paths), RecordingState()
    refusal = _can_record(app, arm_status)
    if refusal is not None:
        return with_notice(app, refusal, now, level="warning"), recording
    logger.info("Recording the release motion")
    return with_notice(app, NOTICE_RECORDING, now), RecordingState(active=True, started_at=now)


def _record_frame(
    app: AppState, recording: RecordingState, commanded: Mapping[str, float], arm_status: str, now: float,
    paths: ArmDataPaths, max_s: float = RECORD_MAX_S,
) -> tuple[AppState, RecordingState]:
    if not recording.active:
        return app, recording
    if _can_record(app, arm_status) is not None:
        logger.warning("Release recording cancelled: the arm stopped following")
        if app.stopped:  # keep the STOP notice
            return app, RecordingState()
        return with_notice(app, NOTICE_CANCELLED, now, level="warning"), RecordingState()
    frame = MotionFrame(t=now - recording.started_at, pose={key: float(value) for key, value in commanded.items()})
    recorded = replace(recording, frames=recording.frames + (frame,))
    if frame.t >= max_s:
        logger.info("Release recording reached its maximum length (%.0f s)", max_s)
        return _finish(app, recorded, now, paths, NOTICE_MOTION_SAVED_MAX), RecordingState()
    return app, recorded


def handle_staff_keys(
    app: AppState,
    recording: RecordingState,
    commands: Iterable[str],
    commanded: Mapping[str, float],
    arm_status: str,
    now: float,
    paths: ArmDataPaths = DEFAULT_PATHS,
    max_s: float = RECORD_MAX_S,
) -> tuple[AppState, RecordingState]:
    """Apply this frame's save-home, save-catch, and record keys, then record this frame's commanded pose.

    `commanded` is the arm pose sent this frame and `arm_status` this frame's arm status.
    """
    for command in commands:
        if command == SAVE_HOME_COMMAND:
            app = _save_home(app, commanded, now, paths)
        elif command == SAVE_CATCH_COMMAND:
            app = _save_catch(app, commanded, arm_status, now, paths)
        elif command == TOGGLE_RECORD_COMMAND:
            app, recording = _toggle(app, recording, arm_status, now, paths)
    return _record_frame(app, recording, commanded, arm_status, now, paths, max_s)
