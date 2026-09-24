"""tests/robot/test_arm_follow.py: Hardware-free checks of leader-arm slow engagement and following.

follow_step is pure, so holding on a missing leader, the speed-limited approach, engagement
within tolerance, real-time following, the dt clamp, and the gripper key are verified directly,
as are the shared approach_pose / within helpers used by Auto Release and the synchronized
approach_pose_sync used for every scripted pose move (all keys arrive in the same frame).
"""

import unittest

from robot.arm_follow import ArmFollowState, approach_pose, approach_pose_sync, disengaged, follow_step, within
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


class ApproachHelperTests(unittest.TestCase):
    def test_approach_pose_limits_every_key_and_can_skip_keys(self):
        moved = approach_pose(HOLD, FAR, 0.05, speed=90.0)
        self.assertEqual(set(moved.values()), {4.5})
        partial = approach_pose(HOLD, FAR, 0.05, speed=90.0, keys=ARM_KEYS[:2])
        self.assertEqual(tuple(partial), ARM_KEYS[:2])
        self.assertEqual(approach_pose(HOLD, FAR, 5.0, speed=30.0)["arm_gripper.pos"], 30.0 * 2 / LOOP_HZ)

    def test_within(self):
        self.assertTrue(within(HOLD, {key: 3.0 for key in ARM_KEYS}, 3.0))
        self.assertFalse(within(HOLD, {**HOLD, "arm_gripper.pos": 3.1}, 3.0))
        self.assertTrue(within(HOLD, {**HOLD, "arm_gripper.pos": 50.0}, 3.0, keys=ARM_KEYS[:5]))



class ApproachSyncTests(unittest.TestCase):
    TARGET = {**HOLD, "arm_wrist_flex.pos": 90.0, "arm_shoulder_lift.pos": 9.0}

    def test_small_delta_moves_the_same_fraction_as_the_large_one(self):
        moved = approach_pose_sync(HOLD, self.TARGET, 0.05, SPEED)
        wrist, shoulder = moved["arm_wrist_flex.pos"], moved["arm_shoulder_lift.pos"]
        self.assertAlmostEqual(wrist, SPEED * 0.05)  # the largest delta moves at the full speed
        self.assertAlmostEqual(shoulder, wrist / 10)  # 9 deg of 90: exactly a tenth of the step
        self.assertEqual(set(moved), set(ARM_KEYS))

    def test_all_keys_arrive_in_the_same_frame(self):
        pose, arrivals = dict(HOLD), {}
        for frame in range(1, 400):
            pose = approach_pose_sync(pose, self.TARGET, 1 / LOOP_HZ, SPEED)
            for key in ("arm_wrist_flex.pos", "arm_shoulder_lift.pos"):
                if key not in arrivals and pose[key] == self.TARGET[key]:
                    arrivals[key] = frame
            if len(arrivals) == 2:
                break
        self.assertEqual(arrivals["arm_wrist_flex.pos"], arrivals["arm_shoulder_lift.pos"])
        self.assertEqual(arrivals["arm_wrist_flex.pos"], 90)  # 90 deg at 30 deg/s and 30 Hz: 90 frames
        self.assertEqual(pose, self.TARGET)  # the last frame lands exactly

    def test_zero_delta_dt_clamp_and_excluded_keys(self):
        self.assertEqual(approach_pose_sync(HOLD, HOLD, 0.1, SPEED), HOLD)
        clamped = approach_pose_sync(HOLD, self.TARGET, 5.0, SPEED)
        self.assertAlmostEqual(clamped["arm_wrist_flex.pos"], SPEED * 2 / LOOP_HZ)
        held = approach_pose_sync({**HOLD, "arm_gripper.pos": 7.0}, {**self.TARGET, "arm_gripper.pos": 50.0}, 0.05,
                                  SPEED, ARM_KEYS[:5])
        self.assertEqual(held["arm_gripper.pos"], 7.0)  # a held gripper keeps its value


if __name__ == "__main__":
    unittest.main()
