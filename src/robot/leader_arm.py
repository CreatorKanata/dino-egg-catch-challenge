"""src/robot/leader_arm.py: SO-100/101 leader arm on the Mac, read for Manual Mode puppeteering.

Wraps LeRobot's SO100Leader (imported lazily through a factory, like the other adapters, so
tests use a fake). get_action() reports joints without the `arm_` prefix; read_pose() adds it
and returns exactly the six ARM_KEYS. A failed read returns None instead of raising, so the
control loop holds the arm and keeps driving the base (docs/spec/operating-modes.md, section 3).
"""

from collections.abc import Callable
import logging
from typing import Any

from robot.config import ARM_KEYS, LEADER_ARM_ID, LEADER_ARM_PORT, LEADER_KEYS

logger = logging.getLogger(__name__)


def _make_so100_leader(port: str, teleop_id: str) -> Any:
    # Lazy import keeps this module importable (and testable) without LeRobot.
    from lerobot.teleoperators.so_leader import SO100Leader, SO100LeaderConfig

    return SO100Leader(SO100LeaderConfig(port=port, id=teleop_id))


def to_arm_pose(action: Any) -> dict[str, float]:
    """Leader keys (`shoulder_pan.pos`, ...) -> the six `arm_*` keys as floats.

    Raises KeyError for a missing joint and TypeError/ValueError for a non-numeric value.
    """
    return {arm_key: float(action[leader_key]) for arm_key, leader_key in zip(ARM_KEYS, LEADER_KEYS)}


class LeaderArm:
    """connect(), read_pose() once per loop frame (None on failure), disconnect()."""

    def __init__(
        self,
        port: str = LEADER_ARM_PORT,
        leader_id: str = LEADER_ARM_ID,
        teleop_factory: Callable[[str, str], Any] = _make_so100_leader,
    ) -> None:
        self._teleop = teleop_factory(port, leader_id)
        self._port = port
        self._faulted = False

    @property
    def is_connected(self) -> bool:
        return bool(self._teleop.is_connected)

    def connect(self) -> None:
        """Open the leader bus with its stored calibration; errors propagate to the caller."""
        self._teleop.connect()
        logger.info("Leader arm connected on %s", self._port)

    def read_pose(self) -> dict[str, float] | None:
        """The leader pose with `arm_` keys, or None when the read fails (never raises)."""
        try:
            pose = to_arm_pose(self._teleop.get_action())
        except Exception as error:  # bus errors, disconnects, malformed readings
            if not self._faulted:
                logger.warning("Leader arm read failed; holding the arm: %s", error)
            self._faulted = True
            return None
        if self._faulted:
            logger.info("Leader arm readings recovered")
        self._faulted = False
        return pose

    def disconnect(self) -> None:
        """Close the leader bus. Safe to call when not connected."""
        if not self.is_connected:
            return
        self._teleop.disconnect()
        logger.info("Leader arm disconnected")
