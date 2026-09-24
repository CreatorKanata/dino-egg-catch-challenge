"""tests/robot/test_episode_gate.py: Checks of the recorder's pure start-pose gate.

The leader must be within GATE_TOLERANCE_DEG of the catch pose on every non-gripper joint; the
gripper is ignored. Also covers the worst-joint report, the rate-limited announcement, and the
stop / ready / timeout / wait decision. No LeRobot and no hardware.
"""

import math
import unittest

from robot.config import ARM_KEYS
from robot.recording.config_recording import GATE_TOLERANCE_DEG
from robot.recording.episode_gate import (
    GATE_KEYS,
    LEADER_UNREADABLE,
    GateStatus,
    announcement,
    gate_decision,
    gate_status,
    joint_label,
)

CATCH = {key: float(index * 10) for index, key in enumerate(ARM_KEYS)}


def leader(**offsets):
    return {key: CATCH[key] + offsets.get(key.removeprefix("arm_").removesuffix(".pos"), 0.0) for key in ARM_KEYS}


class GateStatusTests(unittest.TestCase):
    def test_gripper_is_not_a_gate_key(self):
        self.assertNotIn("arm_gripper.pos", GATE_KEYS)
        self.assertEqual(len(GATE_KEYS), 5)

    def test_ready_at_the_catch_pose_with_the_gripper_ignored(self):
        status = gate_status(leader(gripper=80.0), CATCH)
        self.assertTrue(status.ready)

    def test_ready_at_the_tolerance_boundary(self):
        self.assertTrue(gate_status(leader(elbow_flex=GATE_TOLERANCE_DEG), CATCH).ready)

    def test_not_ready_reports_the_worst_joint(self):
        status = gate_status(leader(shoulder_lift=12.0, wrist_flex=-25.5), CATCH)
        self.assertEqual(status, GateStatus(ready=False, worst_joint="arm_wrist_flex.pos", worst_error_deg=25.5))

    def test_custom_tolerance(self):
        self.assertFalse(gate_status(leader(wrist_roll=4.0), CATCH, tolerance=3.0).ready)

    def test_joint_label(self):
        self.assertEqual(joint_label("arm_wrist_flex.pos"), "wrist flex")


class AnnouncementTests(unittest.TestCase):
    NOT_READY = GateStatus(ready=False, worst_joint="arm_elbow_flex.pos", worst_error_deg=23.4)

    def test_first_announcement_names_the_worst_joint(self):
        text = announcement(self.NOT_READY, now=10.0, last_spoken_at=None, interval_s=3.0)
        self.assertEqual(text, "Move the leader to the catch pose: elbow flex off by 23 degrees")

    def test_rate_limited(self):
        self.assertIsNone(announcement(self.NOT_READY, now=12.9, last_spoken_at=10.0, interval_s=3.0))
        self.assertIsNotNone(announcement(self.NOT_READY, now=13.0, last_spoken_at=10.0, interval_s=3.0))

    def test_silent_when_ready(self):
        ready = GateStatus(ready=True, worst_joint="arm_elbow_flex.pos", worst_error_deg=1.0)
        self.assertIsNone(announcement(ready, now=10.0, last_spoken_at=None, interval_s=3.0))

    def test_unreadable_leader(self):
        self.assertFalse(LEADER_UNREADABLE.ready)
        self.assertTrue(math.isinf(LEADER_UNREADABLE.worst_error_deg))
        text = announcement(LEADER_UNREADABLE, now=0.0, last_spoken_at=None, interval_s=3.0)
        self.assertEqual(text, "Leader arm not readable")


class DecisionTests(unittest.TestCase):
    READY = GateStatus(ready=True, worst_joint="arm_elbow_flex.pos", worst_error_deg=1.0)
    WAIT = GateStatus(ready=False, worst_joint="arm_elbow_flex.pos", worst_error_deg=20.0)

    def test_wait_before_the_timeout(self):
        self.assertEqual(gate_decision(self.WAIT, started_at=0.0, now=119.9, timeout_s=120.0), "wait")

    def test_timeout(self):
        self.assertEqual(gate_decision(self.WAIT, started_at=0.0, now=120.0, timeout_s=120.0), "timeout")

    def test_ready_wins_over_timeout(self):
        self.assertEqual(gate_decision(self.READY, started_at=0.0, now=500.0, timeout_s=120.0), "ready")

    def test_stop_wins_over_everything(self):
        self.assertEqual(gate_decision(self.READY, 0.0, 1.0, 120.0, stop_requested=True), "stop")


if __name__ == "__main__":
    unittest.main()
