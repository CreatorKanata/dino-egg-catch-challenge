"""src/robot/recording/lerobot_io.py: LeRobot-facing pieces of the pick_egg recorder.

Creates the LeKiwi client, SO100 leader, and keyboard teleoperator, creates or resumes the
LeRobotDataset with the robot's action/observation features, and wraps the fork's record_loop
into the callables episode_loop.py runs, following examples/lekiwi/record.py and
scripts/lerobot_record.py. Every LeRobot import happens inside a function, so `--help`, the rest
of the application, and the tests never load LeRobot, `datasets`, or `av`.
Shutdown order: zero base and disconnect the robot first (never leave the base moving), then the
leader, keyboard, listener, and Rerun, then finalize the dataset and push it.
"""

from collections.abc import Callable, Mapping, MutableMapping
import logging
from pathlib import Path
from typing import Any

from robot.config import LEADER_ARM_ID, ROBOT_ID
from robot.leader_arm import LeaderArm
from robot.recording.config_recording import IMAGE_WRITER_THREADS, RERUN_SESSION_NAME
from robot.recording.episode_loop import SessionIO
from robot.recording.gate_runner import run_gate, zero_base
from robot.recording.session import SessionPlan

logger = logging.getLogger(__name__)


def lerobot_home() -> Path:
    from lerobot.utils.constants import HF_LEROBOT_HOME

    return Path(HF_LEROBOT_HOME)


def make_devices(remote_ip: str, leader_port: str) -> tuple[Any, Any, Any]:
    """The LeKiwi client, SO100 leader, and keyboard teleoperator, not yet connected."""
    from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
    from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
    from lerobot.teleoperators.so_leader import SO100Leader, SO100LeaderConfig

    return (LeKiwiClient(LeKiwiClientConfig(remote_ip=remote_ip, id=ROBOT_ID)),
            SO100Leader(SO100LeaderConfig(port=leader_port, id=LEADER_ARM_ID)),
            KeyboardTeleop(KeyboardTeleopConfig()))


def open_dataset(robot: Any, plan: SessionPlan, root: Path, resume: bool) -> Any:
    """Create the dataset at `root` (as the fork's LeKiwi example) or resume it (as lerobot_record.py)."""
    from lerobot.common.control_utils import sanity_check_dataset_robot_compatibility
    from lerobot.datasets import LeRobotDataset
    from lerobot.utils.constants import ACTION, OBS_STR
    from lerobot.utils.feature_utils import hw_to_dataset_features

    features = {**hw_to_dataset_features(robot.action_features, ACTION),
                **hw_to_dataset_features(robot.observation_features, OBS_STR)}
    if resume:
        dataset = LeRobotDataset.resume(plan.repo_id, root=root, image_writer_threads=IMAGE_WRITER_THREADS)
        sanity_check_dataset_robot_compatibility(dataset, robot, plan.fps, features)
        return dataset
    return LeRobotDataset.create(repo_id=plan.repo_id, fps=plan.fps, features=features, root=root,
                                 robot_type=robot.name, use_videos=True, image_writer_threads=IMAGE_WRITER_THREADS)


def start_input(display: bool) -> tuple[Any, MutableMapping[str, bool]]:
    """The fork's keyboard listener and events dict, and Rerun when `display`."""
    from lerobot.utils.keyboard_input import init_keyboard_listener

    listener, events = init_keyboard_listener()
    if display:
        from lerobot.utils.visualization_utils import init_rerun

        init_rerun(session_name=RERUN_SESSION_NAME)
    return listener, events


def speaker() -> Callable[[str], None]:
    from lerobot.utils.utils import log_say

    return lambda text: log_say(text, play_sounds=True)


def make_session_io(plan: SessionPlan, robot: Any, leader: Any, keyboard: Any, events: MutableMapping[str, bool],
                    dataset: Any, catch_pose: Mapping[str, float], display: bool) -> SessionIO:
    """Wrap the fork's record_loop and dataset calls for episode_loop.run_session."""
    import time

    from lerobot.processor import make_default_processors
    from lerobot.scripts.lerobot_record import record_loop

    teleop_proc, robot_proc, obs_proc = make_default_processors()
    say = speaker()
    gate_leader = LeaderArm(teleop_factory=lambda _port, _id: leader)  # read_pose(): arm_ keys or None

    def phase(with_dataset: bool, seconds: float) -> None:
        record_loop(robot=robot, events=events, fps=plan.fps, teleop_action_processor=teleop_proc,
                    robot_action_processor=robot_proc, robot_observation_processor=obs_proc,
                    dataset=dataset if with_dataset else None, teleop=[leader, keyboard],
                    control_time_s=seconds, single_task=plan.single_task, display_data=display)

    return SessionIO(
        events=events,
        gate=lambda: run_gate(robot, gate_leader.read_pose, catch_pose, events, plan.fps, say),
        record_episode=lambda: phase(True, plan.episode_time_s),
        reset=lambda: phase(False, plan.reset_time_s),
        save_episode=lambda: _save(dataset),
        discard_episode=dataset.clear_episode_buffer,
        zero_base=lambda: zero_base(robot),
        say=say,
        now=time.monotonic,
    )


def _save(dataset: Any) -> None:
    if dataset.has_pending_frames():
        dataset.save_episode()
    else:
        logger.warning("Episode has no frames; nothing saved")


def _attempt(label: str, action: Callable[[], None]) -> bool:
    try:
        action()
        return True
    except Exception:
        logger.exception("Shutdown step failed: %s", label)
        return False


def _disconnect(device: Any) -> None:
    if device is not None and device.is_connected:
        device.disconnect()


def shutdown(robot: Any, leader: Any, keyboard: Any, listener: Any, dataset: Any, push: bool, display: bool) -> bool:
    """Stop everything in a safe order; return True when the dataset was pushed to the Hub."""
    if robot is not None and robot.is_connected:
        _attempt("zero base velocities", lambda: zero_base(robot))
    _attempt("robot disconnect", lambda: _disconnect(robot))
    _attempt("leader disconnect", lambda: _disconnect(leader))
    _attempt("keyboard disconnect", lambda: _disconnect(keyboard))
    if listener is not None:
        _attempt("keyboard listener stop", listener.stop)
    if display:
        from lerobot.utils.visualization_utils import shutdown_rerun

        _attempt("Rerun shutdown", shutdown_rerun)
    if dataset is None:
        return False
    if dataset.has_pending_frames():
        _attempt("discard unsaved episode", dataset.clear_episode_buffer)
    if not _attempt("dataset finalize", dataset.finalize) or not push:
        return False
    if dataset.num_episodes == 0:
        logger.warning("No episodes saved; skipping the push to the Hub")
        return False
    logger.info("Pushing %s to the Hub", dataset.repo_id)
    return _attempt("push to the Hub", dataset.push_to_hub)
