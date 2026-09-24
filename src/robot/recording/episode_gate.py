"""src/robot/recording/episode_gate.py: Pure start-pose gate run before every recorded episode.

Every episode must start from the catch pose (head down). The follower is driven there first;
then the leader must be within GATE_TOLERANCE_DEG of the catch pose on every non-gripper joint,
so record_loop does not jump the follower to a distant leader pose. The gripper is ignored
because the mouth may be at any opening. While waiting, a spoken hint names the worst joint at
most every GATE_ANNOUNCE_INTERVAL_S. Stdlib-only; the caller passes the time.
"""

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
import math
from typing import Final, Literal

from robot.config import ARM_KEYS
from robot.recording.config_recording import GATE_ANNOUNCE_INTERVAL_S, GATE_TOLERANCE_DEG

GATE_KEYS: Final = tuple(key for key in ARM_KEYS if key != "arm_gripper.pos")

GateDecision = Literal["ready", "wait", "timeout", "stop"]


@dataclass(frozen=True)
class GateStatus:
    """Whether the leader is at the catch pose, and the joint furthest from it (degrees)."""

    ready: bool
    worst_joint: str
    worst_error_deg: float


# The leader's read failed this frame: never ready, announced as such.
LEADER_UNREADABLE: Final = GateStatus(ready=False, worst_joint="", worst_error_deg=math.inf)


def gate_status(
    leader_pose: Mapping[str, float],
    catch_pose: Mapping[str, float],
    tolerance: float = GATE_TOLERANCE_DEG,
    keys: Iterable[str] = GATE_KEYS,
) -> GateStatus:
    """Compare the leader's `arm_*` pose with the catch pose on `keys` (non-gripper joints by default)."""
    errors = {key: abs(float(leader_pose[key]) - float(catch_pose[key])) for key in keys}
    worst = max(errors, key=lambda key: errors[key])
    return GateStatus(ready=errors[worst] <= tolerance, worst_joint=worst, worst_error_deg=errors[worst])


def joint_label(key: str) -> str:
    """`arm_wrist_flex.pos` -> `wrist flex`, for speech."""
    return key.removeprefix("arm_").removesuffix(".pos").replace("_", " ")


def announcement(
    status: GateStatus,
    now: float,
    last_spoken_at: float | None,
    interval_s: float = GATE_ANNOUNCE_INTERVAL_S,
) -> str | None:
    """The hint to speak now, or None when ready or when the last hint was under `interval_s` ago."""
    if status.ready:
        return None
    if last_spoken_at is not None and now - last_spoken_at < interval_s:
        return None
    if not status.worst_joint:
        return "Leader arm not readable"
    return (f"Move the leader to the catch pose: {joint_label(status.worst_joint)} "
            f"off by {round(status.worst_error_deg)} degrees")


def gate_decision(
    status: GateStatus,
    started_at: float,
    now: float,
    timeout_s: float,
    stop_requested: bool = False,
) -> GateDecision:
    """stop (keyboard stop_recording) > ready > timeout (skip the episode) > wait."""
    if stop_requested:
        return "stop"
    if status.ready:
        return "ready"
    if now - started_at >= timeout_s:
        return "timeout"
    return "wait"
