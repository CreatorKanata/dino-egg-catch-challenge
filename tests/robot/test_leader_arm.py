"""tests/robot/test_leader_arm.py: Hardware-free checks of the leader-arm adapter.

A fake teleoperator replaces SO100Leader, so the `arm_` key mapping, the None-on-failure
contract with recovery, the once-per-streak logging, and connect/disconnect are verified
without a serial port.
"""

import unittest

from robot.config import ARM_KEYS, LEADER_KEYS
from robot.leader_arm import LeaderArm, to_arm_pose

READING = {key: float(index) * 10 for index, key in enumerate(LEADER_KEYS)}


class FakeTeleop:
    """Mimics SO100Leader: connect, get_action (unprefixed keys), disconnect, is_connected."""

    def __init__(self, readings):
        self.readings = list(readings)
        self.is_connected = False
        self.calls = 0

    def connect(self):
        self.is_connected = True

    def get_action(self):
        self.calls += 1
        reading = self.readings.pop(0) if len(self.readings) > 1 else self.readings[0]
        if isinstance(reading, Exception):
            raise reading
        return dict(reading)

    def disconnect(self):
        self.is_connected = False


def make_leader(*readings):
    teleop = FakeTeleop(readings or (READING,))
    created = []

    def factory(port, leader_id):
        created.append((port, leader_id))
        return teleop

    return LeaderArm(port="/dev/fake", leader_id="dino_leader_arm", teleop_factory=factory), teleop, created


class LeaderArmTests(unittest.TestCase):
    def test_factory_receives_port_and_id(self):
        _, _, created = make_leader()
        self.assertEqual(created, [("/dev/fake", "dino_leader_arm")])

    def test_read_pose_returns_the_six_prefixed_keys(self):
        leader, _, _ = make_leader({**READING, "extra.pos": 1.0})
        pose = leader.read_pose()
        self.assertEqual(list(pose), list(ARM_KEYS))
        self.assertEqual(pose["arm_gripper.pos"], READING["gripper.pos"])
        self.assertTrue(all(type(value) is float for value in pose.values()))

    def test_failure_returns_none_and_recovers(self):
        missing = {key: value for key, value in READING.items() if key != "elbow_flex.pos"}
        leader, _, _ = make_leader(ConnectionError("Incorrect status packet!"), missing,
                                   {**READING, "gripper.pos": "x"}, READING)
        with self.assertLogs("robot.leader_arm", level="INFO") as logs:
            results = [leader.read_pose() for _ in range(4)]
        self.assertEqual(results[:3], [None, None, None])
        self.assertEqual(set(results[3]), set(ARM_KEYS))
        levels = [record.levelname for record in logs.records]
        self.assertEqual(levels, ["WARNING", "INFO"])  # once per streak, then recovery

    def test_connect_and_disconnect(self):
        leader, teleop, _ = make_leader()
        self.assertFalse(leader.is_connected)
        leader.disconnect()  # safe before connect
        leader.connect()
        self.assertTrue(leader.is_connected and teleop.is_connected)
        leader.disconnect()
        self.assertFalse(teleop.is_connected)

    def test_to_arm_pose_rejects_missing_keys(self):
        with self.assertRaises(KeyError):
            to_arm_pose({})


if __name__ == "__main__":
    unittest.main()
