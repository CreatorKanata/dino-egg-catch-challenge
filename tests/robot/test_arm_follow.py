"""tests/robot/test_arm_follow.py: Hardware-free checks of leader-arm slow engagement and following.

follow_step is pure, so holding on a missing leader, the speed-limited approach, engagement
within tolerance, real-time following, the dt clamp, and the gripper key are verified directly.
"""

import unittest

from robot.arm_follow import ArmFollowState, disengaged, follow_step
from robot.config import ARM_KEYS, LOOP_HZ

HOLD = {key: 0.0 for key in ARM_KEYS}
FAR = {key: 90.0 for key in ARM_KEYS}
SPEED, TOLERANCE = 30.0, 3.0


def step(commanded, leader, state=ArmFollowState(), dt=0.1):
    return follow_step(commanded, leader, state, dt, speed=SPEED, tolerance=TOLERANCE)


class FollowStepTests(unittest.TestCase):
    def test_holds_commanded_pose_without_leader(self):
        for state in (ArmFollowState(), ArmFollowState(engaged=True)):
            with self.subTest(state=state):
                pose, next_state = step(HOLD, None, state)
                self.assertEqual((pose, next_state), (HOLD, state))
                self.assertIsNot(pose, HOLD)

    def test_approach_is_limited_by_speed_times_dt(self):
        pose, state = step(HOLD, {**FAR, "arm_elbow_flex.pos": -90.0}, dt=0.05)
        self.assertFalse(state.engaged)
        self.assertAlmostEqual(pose["arm_shoulder_pan.pos"], 1.5)
        self.assertAlmostEqual(pose["arm_elbow_flex.pos"], -1.5)
        self.assertAlmostEqual(pose["arm_gripper.pos"], 1.5)  # gripper (percent) treated the same

    def test_close_keys_do_not_overshoot(self):
        leader = {**FAR, "arm_wrist_roll.pos": 1.0}
        pose, _ = step(HOLD, leader, dt=0.05)
        self.assertEqual(pose["arm_wrist_roll.pos"], 1.0)

    def test_engages_when_every_key_is_within_tolerance(self):
        leader = {key: 4.0 for key in ARM_KEYS}
        pose, state = step(HOLD, leader, dt=0.05)  # moves 1.5 -> 2.5 away, within 3.0
        self.assertTrue(state.engaged)
        self.assertEqual(pose, leader)

    def test_gripper_alone_can_block_engagement(self):
        leader = {**HOLD, "arm_gripper.pos": 10.0}
        pose, state = step(HOLD, leader, dt=0.05)
        self.assertFalse(state.engaged)
        self.assertAlmostEqual(pose["arm_gripper.pos"], 1.5)

    def test_first_frame_with_zero_dt_engages_only_when_already_close(self):
        self.assertTrue(step(HOLD, {key: 2.0 for key in ARM_KEYS}, dt=0.0)[1].engaged)
        pose, state = step(HOLD, FAR, dt=0.0)
        self.assertEqual((pose, state.engaged), (HOLD, False))

    def test_following_after_engaged_tracks_leader_exactly(self):
        leader = {**FAR, "extra.pos": 1.0}
        pose, state = step(HOLD, leader, ArmFollowState(engaged=True))
        self.assertTrue(state.engaged)
        self.assertEqual(pose, FAR)  # only ARM_KEYS, jumps allowed once engaged

    def test_dt_is_clamped(self):
        pose, _ = step(HOLD, FAR, dt=5.0)
        self.assertAlmostEqual(pose["arm_shoulder_lift.pos"], SPEED * 2 / LOOP_HZ)
        pose, _ = step(HOLD, FAR, dt=-1.0)
        self.assertEqual(pose, HOLD)

    def test_disengaged_helper(self):
        self.assertEqual(disengaged(), ArmFollowState(engaged=False))


if __name__ == "__main__":
    unittest.main()
