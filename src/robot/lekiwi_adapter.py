"""src/robot/lekiwi_adapter.py: LeKiwiClient wrapper for Manual Mode base and arm commands.

The fork's host writes Goal_Position on every action and fails when no arm keys are present,
so each command carries six arm positions plus the three base velocities. The arm pose is
captured at connect and replaced by every explicitly commanded pose (leader-arm following);
otherwise the last pose is repeated. Arm values are never derived from controller input.
Every action also carries ARM_TORQUE_KEY (1.0 torque on, 0.0 released; read by the fork's host),
and capture_hold re-reads the held pose after the arm was moved by hand while its torque was off.
Disconnecting always sends zero velocities first; the host's 500 ms watchdog is only a backstop.
"""

import logging
from collections.abc import Callable, Mapping
from typing import Any

from robot.config import ARM_KEYS, ARM_TORQUE_KEY, PI_REMOTE_IP, ROBOT_ID, ZMQ_CMD_PORT, ZMQ_OBSERVATION_PORT
from robot.controller_to_action import BASE_KEYS, stop_action

logger = logging.getLogger(__name__)


def _make_lekiwi_client(**config: Any) -> Any:
    # Lazy import keeps the pure helpers below testable without LeRobot.
    from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig

    return LeKiwiClient(LeKiwiClientConfig(**config))


def capture_arm_pose(observation: Mapping[str, Any]) -> dict[str, float]:
    """Return the six arm positions from an observation as floats.

    Raises KeyError when an arm key is missing and ValueError when a value is not numeric.
    """
    missing = [key for key in ARM_KEYS if key not in observation]
    if missing:
        raise KeyError(f"Observation lacks arm positions: {', '.join(missing)}")
    try:
        return {key: float(observation[key]) for key in ARM_KEYS}
    except (TypeError, ValueError) as error:
        raise ValueError(f"Arm position is not numeric: {error}") from error


def compose_action(base: Mapping[str, float], arm_hold: Mapping[str, float]) -> dict[str, float]:
    """Exactly the six held arm keys plus the three base keys; anything else is dropped."""
    return {
        **{key: float(arm_hold[key]) for key in ARM_KEYS},
        **{key: float(base[key]) for key in BASE_KEYS},
    }


class LeKiwiAdapter:
    """Connect to the LeKiwi host, hold the last commanded arm pose, and send base velocities."""

    def __init__(
        self,
        remote_ip: str = PI_REMOTE_IP,
        robot_id: str = ROBOT_ID,
        port_zmq_cmd: int = ZMQ_CMD_PORT,
        port_zmq_observations: int = ZMQ_OBSERVATION_PORT,
        client_factory: Callable[..., Any] = _make_lekiwi_client,
    ) -> None:
        self._client = client_factory(
            remote_ip=remote_ip,
            id=robot_id,
            port_zmq_cmd=port_zmq_cmd,
            port_zmq_observations=port_zmq_observations,
        )
        self._remote_ip = remote_ip
        self.arm_hold: dict[str, float] | None = None

    @property
    def is_connected(self) -> bool:
        return bool(self._client.is_connected)

    def connect(self) -> None:
        """Open the ZMQ link, then capture the arm pose to hold; fail if it is unknown."""
        self._client.connect()
        logger.info("Connected to LeKiwi host at %s", self._remote_ip)
        try:
            self.arm_hold = capture_arm_pose(self._client.get_observation())
        except (KeyError, ValueError):
            logger.exception("Cannot read the arm pose; Manual Mode will not start")
            raise
        logger.info("Holding arm at startup pose: %s", self.arm_hold)

    def observe(self) -> dict[str, Any]:
        return self._client.get_observation()

    def send_action(
        self, base: Mapping[str, float], arm_pose: Mapping[str, float] | None = None, arm_torque: bool = True
    ) -> None:
        """Send the base velocities with `arm_pose`, or with the held pose when it is None, and the
        arm torque flag (ARM_TORQUE_KEY: 1.0 on, 0.0 released).

        A given `arm_pose` becomes the new held pose (a new dict; the old one is not mutated),
        so stop() and later frames repeat the last commanded arm position.
        """
        if self.arm_hold is None:
            raise RuntimeError("Arm pose is unknown; call connect() first")
        pose = self.arm_hold if arm_pose is None else {key: float(arm_pose[key]) for key in ARM_KEYS}
        self._client.send_action({**compose_action(base, pose), ARM_TORQUE_KEY: 1.0 if arm_torque else 0.0})
        self.arm_hold = pose

    def capture_hold(self, observation: Mapping[str, Any]) -> None:
        """Replace the held pose with the observed arm positions (after torque off, the arm may have
        been moved by hand). A bad observation is logged and the old hold kept; never raises."""
        try:
            self.arm_hold = capture_arm_pose(observation)
        except (KeyError, ValueError):
            logger.exception("Cannot re-read the arm pose; keeping the previous hold")
            return
        logger.info("Holding arm at observed pose: %s", self.arm_hold)

    def stop(self, arm_torque: bool = True) -> None:
        """Send zero base velocities (with the held arm pose and the torque flag) when possible."""
        if not self.is_connected:
            return
        if self.arm_hold is None:
            logger.warning("Arm pose unknown; cannot send a stop command, relying on host watchdog")
            return
        self.send_action(stop_action(), arm_torque=arm_torque)

    def disconnect(self) -> None:
        """Stop the base, then close the ZMQ sockets. Safe to call when not connected."""
        if not self.is_connected:
            return
        try:
            self.stop()
        finally:
            self._client.disconnect()
            logger.info("Disconnected from LeKiwi host")
