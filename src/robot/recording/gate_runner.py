"""src/robot/recording/gate_runner.py: Drives the start-pose gate against the robot before each episode.

Moves the follower to the catch pose with approach_pose_sync (all joints arrive together, the
gripper takes the catch pose's value, base velocities zero), then holds it there while waiting
for the leader to reach the catch pose (episode_gate.py decides). Every command goes through
the LeKiwi client at `fps` and carries the six arm keys plus zero base velocities, as the host
requires. The devices, clock, and speech are passed in, so tests use fakes and no LeRobot import
happens here.
"""

from collections.abc import Callable, Mapping
from dataclasses import dataclass
import logging
import time
from typing import Any

from robot.arm_follow import approach_pose_sync
from robot.config import ARM_KEYS
from robot.controller_to_action import stop_action
from robot.lekiwi_adapter import capture_arm_pose, compose_action
from robot.recording.config_recording import (
    GATE_ANNOUNCE_INTERVAL_S,
    GATE_SPEED_DEG_S,
    GATE_TIMEOUT_S,
    GATE_TOLERANCE_DEG,
)
from robot.recording.episode_gate import LEADER_UNREADABLE, GateDecision, announcement, gate_decision, gate_status

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class Clock:
    """Monotonic time and sleep; tests pass a fake that advances on sleep."""

    now: Callable[[], float] = time.monotonic
    sleep: Callable[[float], None] = time.sleep


@dataclass(frozen=True)
class GateLimits:
    """Gate tunables (config_recording.py defaults)."""

    tolerance_deg: float = GATE_TOLERANCE_DEG
    timeout_s: float = GATE_TIMEOUT_S
    announce_interval_s: float = GATE_ANNOUNCE_INTERVAL_S
    speed_deg_s: float = GATE_SPEED_DEG_S


def zero_base(robot: Any, arm_pose: Mapping[str, float] | None = None) -> None:
    """Send zero base velocities with `arm_pose`, or with the observed arm pose (the arm stays put)."""
    pose = arm_pose if arm_pose is not None else capture_arm_pose(robot.get_observation())
    robot.send_action(compose_action(stop_action(), pose))


def _pace(clock: Clock, frame_start: float, fps: int) -> None:
    clock.sleep(max(0.0, 1.0 / fps - (clock.now() - frame_start)))


def move_to_catch_pose(
    robot: Any, catch_pose: Mapping[str, float], events: Mapping[str, bool], fps: int,
    clock: Clock = Clock(), limits: GateLimits = GateLimits(),
) -> bool:
    """Step the follower from its observed pose to the catch pose; False if a stop was requested."""
    target = {key: float(catch_pose[key]) for key in ARM_KEYS}
    commanded = capture_arm_pose(robot.get_observation())
    last = clock.now()
    while not events["stop_recording"]:
        now = clock.now()
        commanded = approach_pose_sync(commanded, target, now - last, limits.speed_deg_s)
        last = now
        robot.send_action(compose_action(stop_action(), commanded))
        if commanded == target:
            return True
        _pace(clock, now, fps)
    return False


def wait_for_leader(
    robot: Any, read_leader: Callable[[], Mapping[str, float] | None], catch_pose: Mapping[str, float],
    events: Mapping[str, bool], fps: int, say: Callable[[str], None],
    clock: Clock = Clock(), limits: GateLimits = GateLimits(),
) -> GateDecision:
    """Hold the follower at the catch pose until the leader is there ("ready"), the timeout
    passes ("timeout"), or the keyboard stop_recording event is set ("stop")."""
    hold = compose_action(stop_action(), {key: float(catch_pose[key]) for key in ARM_KEYS})
    started = clock.now()
    last_spoken: float | None = None
    while True:
        now = clock.now()
        leader = read_leader()
        status = LEADER_UNREADABLE if leader is None else gate_status(leader, catch_pose, limits.tolerance_deg)
        decision = gate_decision(status, started, now, limits.timeout_s, bool(events["stop_recording"]))
        if decision != "wait":
            return decision
        text = announcement(status, now, last_spoken, limits.announce_interval_s)
        if text is not None:
            say(text)
            last_spoken = now
        robot.send_action(hold)
        _pace(clock, now, fps)


def run_gate(
    robot: Any, read_leader: Callable[[], Mapping[str, float] | None], catch_pose: Mapping[str, float],
    events: Mapping[str, bool], fps: int, say: Callable[[str], None],
    clock: Clock = Clock(), limits: GateLimits = GateLimits(),
) -> GateDecision:
    """The whole gate: follower to the catch pose, then wait for the leader. Never "wait"."""
    logger.info("Moving the follower to the catch pose")
    if not move_to_catch_pose(robot, catch_pose, events, fps, clock, limits):
        return "stop"
    return wait_for_leader(robot, read_leader, catch_pose, events, fps, say, clock, limits)
