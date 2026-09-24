"""tests/robot/test_lekiwi_adapter.py: Hardware-free checks of the LeKiwi adapter's action content.

The fork's host rejects actions without arm keys, so every command must carry exactly the six
held arm positions plus the three base velocities. A fake client replaces LeKiwiClient.
"""

import unittest

from robot.config import ARM_KEYS
from robot.lekiwi_adapter import LeKiwiAdapter, capture_arm_pose, compose_action

BASE_KEYS = {"x.vel", "y.vel", "theta.vel"}
POSE = {key: float(index) for index, key in enumerate(ARM_KEYS)}
OBSERVATION = {**POSE, "x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0, "front": object()}


class FakeClient:
    """Records sent actions; mimics LeKiwiClient's connect/observe/send/disconnect."""

    def __init__(self, observation=OBSERVATION):
        self.observation = observation
        self.is_connected = False
        self.sent = []

    def connect(self):
        self.is_connected = True

    def get_observation(self):
        return dict(self.observation)

    def send_action(self, action):
        self.sent.append(action)
        return action

    def disconnect(self):
        self.is_connected = False


def make_adapter(client):
    return LeKiwiAdapter(client_factory=lambda **_: client)


class CaptureArmPoseTests(unittest.TestCase):
    def test_extracts_arm_keys_as_floats(self):
        pose = capture_arm_pose({**OBSERVATION, "arm_gripper.pos": 12})
        self.assertEqual(set(pose), set(ARM_KEYS))
        self.assertIs(type(pose["arm_gripper.pos"]), float)
        self.assertEqual(pose["arm_gripper.pos"], 12.0)

    def test_missing_key_raises(self):
        observation = {key: value for key, value in OBSERVATION.items() if key != "arm_elbow_flex.pos"}
        with self.assertRaisesRegex(KeyError, "arm_elbow_flex.pos"):
            capture_arm_pose(observation)

    def test_non_numeric_value_raises(self):
        with self.assertRaises(ValueError):
            capture_arm_pose({**OBSERVATION, "arm_wrist_roll.pos": "n/a"})


class ComposeActionTests(unittest.TestCase):
    def test_exactly_nine_keys(self):
        base = {"x.vel": 0.1, "y.vel": 0.0, "theta.vel": -30.0, "extra": 5.0}
        action = compose_action(base, {**POSE, "arm_extra.pos": 1.0})
        self.assertEqual(set(action), set(ARM_KEYS) | BASE_KEYS)
        self.assertEqual(action["theta.vel"], -30.0)
        self.assertEqual(action["arm_shoulder_lift.pos"], 1.0)


class AdapterTests(unittest.TestCase):
    def test_connect_captures_pose_and_send_holds_it(self):
        client = FakeClient()
        adapter = make_adapter(client)
        adapter.connect()
        self.assertEqual(adapter.arm_hold, POSE)
        adapter.send_action({"x.vel": 0.2, "y.vel": 0.0, "theta.vel": 0.0})
        self.assertEqual(len(client.sent), 1)
        self.assertEqual(set(client.sent[0]), set(ARM_KEYS) | BASE_KEYS)
        self.assertEqual({key: client.sent[0][key] for key in ARM_KEYS}, POSE)

    def test_connect_without_arm_pose_fails(self):
        client = FakeClient(observation={"x.vel": 0.0})
        with self.assertRaises(KeyError):
            make_adapter(client).connect()

    def test_send_before_connect_raises(self):
        with self.assertRaises(RuntimeError):
            make_adapter(FakeClient()).send_action({"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0})

    def test_disconnect_sends_zeros_with_held_pose_first(self):
        client = FakeClient()
        adapter = make_adapter(client)
        adapter.connect()
        adapter.disconnect()
        self.assertFalse(client.is_connected)
        self.assertEqual(client.sent[-1], {**POSE, "x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0})

    def test_send_with_arm_pose_updates_the_held_pose(self):
        client = FakeClient()
        adapter = make_adapter(client)
        adapter.connect()
        startup = adapter.arm_hold
        leader = {key: value + 10.0 for key, value in POSE.items()}
        adapter.send_action({"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}, {**leader, "extra.pos": 1.0})
        self.assertEqual({key: client.sent[-1][key] for key in ARM_KEYS}, leader)
        self.assertEqual(adapter.arm_hold, leader)
        self.assertEqual(startup, POSE)  # reassigned, not mutated
        adapter.send_action({"x.vel": 0.1, "y.vel": 0.0, "theta.vel": 0.0})  # None -> repeat held pose
        self.assertEqual({key: client.sent[-1][key] for key in ARM_KEYS}, leader)
        adapter.stop()
        self.assertEqual(client.sent[-1], {**leader, "x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0})

    def test_send_with_arm_pose_before_connect_raises(self):
        with self.assertRaises(RuntimeError):
            make_adapter(FakeClient()).send_action({"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}, POSE)

    def test_stop_and_disconnect_are_safe_when_not_connected(self):
        client = FakeClient()
        adapter = make_adapter(client)
        adapter.stop()
        adapter.disconnect()
        self.assertEqual(client.sent, [])


if __name__ == "__main__":
    unittest.main()
