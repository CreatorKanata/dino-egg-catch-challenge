"""src/robot/recording/record_pick_egg.py: CLI that records pick_egg demonstrations into a LeRobot dataset.

`python -m robot.recording.record_pick_egg --egg-color green` (owner decision, 2026-09-25): the fork's
record_loop with [SO100 leader, keyboard] teleoperators drives LeKiwi from the catch pose until the
egg is held; the lerobot-record CLI cannot drive LeKiwi with a lone arm teleoperator. Each episode
starts behind the start-pose gate (gate_runner.py). The Manual Mode application must be closed
because this recorder owns the robot connection. Kept thin: parsing and wiring only; LeRobot is
imported lazily (lerobot_io.py), so `--help` works without it.
"""

import argparse
import logging
import sys

from robot.arm_motions import ArmFileError, load_pose
from robot.arm_store import DEFAULT_PATHS
from robot.config import LEADER_ARM_PORT, PI_REMOTE_IP
from robot.recording.config_recording import (
    DEFAULT_EPISODE_TIME_S,
    DEFAULT_FPS,
    DEFAULT_NUM_EPISODES,
    DEFAULT_REPO_ID,
    DEFAULT_RESET_TIME_S,
    DEFAULT_TASK,
    EGG_COLORS,
    KEY_MAP,
)
from robot.recording.session import SessionPlan, SessionState, plan_from_args, resolve_root, start_error, summary_lines

logger = logging.getLogger(__name__)

STARTUP_NOTE = ("Close the Manual Mode application (robot.teleop_drive) first: this recorder owns the robot "
                "connection. The LeKiwi host must be running on the Pi.")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python -m robot.recording.record_pick_egg",
                                     description="Record pick_egg demonstrations from the catch pose.")
    parser.add_argument("--egg-color", required=True, choices=EGG_COLORS,
                        help="egg used in this session; appended to the task as '(<color> egg)'")
    parser.add_argument("--repo-id", default=DEFAULT_REPO_ID, help=f"Hub dataset (default {DEFAULT_REPO_ID})")
    parser.add_argument("--num-episodes", type=int, default=DEFAULT_NUM_EPISODES,
                        help=f"episode attempts this session (default {DEFAULT_NUM_EPISODES})")
    parser.add_argument("--episode-time-s", type=float, default=DEFAULT_EPISODE_TIME_S,
                        help=f"longest episode in seconds (default {DEFAULT_EPISODE_TIME_S:g})")
    parser.add_argument("--reset-time-s", type=float, default=DEFAULT_RESET_TIME_S,
                        help=f"unrecorded reset phase in seconds (default {DEFAULT_RESET_TIME_S:g})")
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS, help=f"frame rate (default {DEFAULT_FPS})")
    parser.add_argument("--task", default=DEFAULT_TASK, help=f"task text (default '{DEFAULT_TASK}')")
    parser.add_argument("--no-push", action="store_true", help="do not upload the dataset to the Hub at the end")
    parser.add_argument("--resume", action="store_true", help="append episodes to the existing local dataset")
    parser.add_argument("--root", default=None, help="dataset directory (default: LeRobot's cache/<repo-id>)")
    parser.add_argument("--remote-ip", default=PI_REMOTE_IP, help=f"LeKiwi host address (default {PI_REMOTE_IP})")
    parser.add_argument("--leader-port", default=LEADER_ARM_PORT,
                        help=f"leader arm serial port (default {LEADER_ARM_PORT})")
    parser.add_argument("--no-rerun", action="store_true", help="do not open the Rerun viewer")
    return parser


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    return build_parser().parse_args(argv)


def record(args: argparse.Namespace, plan: SessionPlan) -> int:
    """Connect, record, and always shut down; return the exit code."""
    from robot.recording import lerobot_io as io
    from robot.recording.episode_loop import run_session

    try:
        catch_pose = load_pose(DEFAULT_PATHS.catch)
    except (OSError, ArmFileError) as error:
        logger.error("Catch pose missing or invalid at %s (%s); record it with the staff key first",
                     DEFAULT_PATHS.catch, error)
        return 2
    root = resolve_root(plan.repo_id, args.root, io.lerobot_home())
    problem = start_error(args.resume, root.exists(), root)
    if problem is not None:
        logger.error(problem)
        return 2
    display = not args.no_rerun
    robot, leader, keyboard = io.make_devices(args.remote_ip, args.leader_port)
    dataset = listener = None
    display_started = False
    state = SessionState()
    try:
        dataset = io.open_dataset(robot, plan, root, args.resume)
        leader.connect()  # teleoperators before the robot, as lerobot_record.py does
        keyboard.connect()
        robot.connect()
        listener, events = io.start_input(display)
        display_started = display
        state = run_session(plan, io.make_session_io(plan, robot, leader, keyboard, events, dataset,
                                                     catch_pose, display))
    except KeyboardInterrupt:
        logger.warning("Ctrl+C during start-up; shutting down")
        state = SessionState(stopped=True)
    finally:
        pushed = io.shutdown(robot, leader, keyboard, listener, dataset, plan.push, display_started)
        print("\n".join(summary_lines(plan, state, root, pushed)))
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        plan = plan_from_args(args)
    except ValueError as error:
        parser.error(str(error))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    print(STARTUP_NOTE)
    print(f"Robot: {args.remote_ip}  Leader arm: {args.leader_port}  Dataset: {plan.repo_id}  "
          f"Task: '{plan.single_task}'  Push: {'yes' if plan.push else 'no (--no-push)'}")
    print("\n".join(KEY_MAP))
    return record(args, plan)


if __name__ == "__main__":
    sys.exit(main())
