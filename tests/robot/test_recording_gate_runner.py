"""tests/robot/test_recording_gate_runner.py: Checks of the start-pose gate loop with a fake robot and clock.

The follower moves to the catch pose with every joint arriving in the same frame, zero base
velocities and nine keys in every command; the wait holds the catch pose, speaks rate-limited
hints, and ends ready, on timeout, or on a keyboard stop. No LeRobot and no hardware.
"""

import unittest

from robot.config import ARM_KEYS
from robot.controller_to_action import BASE_KEYS
from robot.recording.gate_runner import GateLimits, move_to_catch_pose, run_gate, wait_for_leader, zero_base

from recording_fakes import CATCH, FakeClock, FakeRobot

START = {key: CATCH[key] + (40.0 if key == "arm_shoulder_lift.pos" else 10.0) for key in ARM_KEYS}


def events(stop=False):
    return {"exit_early": False, "rerecord_episode": False, "stop_recording": stop}


def off_by(degrees, key="arm_elbow_flex.pos"):
    return {**CATCH, key: CATCH[key] + degrees}


class MoveTests(unittest.TestCase):
    def test_moves_in_sync_to_the_catch_pose_with_zero_base(self):
        robot = FakeRobot(arm=START)
        self.assertTrue(move_to_catch_pose(robot, CATCH, events(), 30, FakeClock()))
        final = robot.sent[-1]
        self.assertEqual({key: final[key] for key in ARM_KEYS}, CATCH)
        for action in robot.sent:
            self.assertEqual(set(action), set(ARM_KEYS) | set(BASE_KEYS))
            self.assertEqual([action[key] for key in BASE_KEYS], [0.0, 0.0, 0.0])
        middle = robot.sent[len(robot.sent) // 2]
        progress = [(START[k] - middle[k]) / (START[k] - CATCH[k]) for k in ARM_KEYS]
        self.assertTrue(all(abs(value - progress[0]) < 1e-9 for value in progress), progress)

    def test_speed_is_limited(self):
        robot = FakeRobot(arm=START)
        move_to_catch_pose(robot, CATCH, events(), 30, FakeClock(), GateLimits(speed_deg_s=30.0))
        steps = [abs(b["arm_shoulder_lift.pos"] - a["arm_shoulder_lift.pos"]) for a, b in zip(robot.sent, robot.sent[1:])]
        self.assertLessEqual(max(steps), 30.0 / 30 * 2 + 1e-9)
        self.assertGreater(len(robot.sent), 30)  # 40 degrees at 30 deg/s takes over a second

    def test_stop_aborts_the_move(self):
        robot = FakeRobot(arm=START)
        self.assertFalse(move_to_catch_pose(robot, CATCH, events(stop=True), 30, FakeClock()))
        self.assertEqual(robot.sent, [])


class WaitTests(unittest.TestCase):
    def test_ready_when_the_leader_is_close_with_the_gripper_ignored(self):
        robot, spoken = FakeRobot(), []
        leader = {**off_by(5.0), "arm_gripper.pos": 90.0}
        self.assertEqual(wait_for_leader(robot, lambda: leader, CATCH, events(), 30, spoken.append, FakeClock()), "ready")
        self.assertEqual(spoken, [])

    def test_waits_holding_the_catch_pose_then_ready(self):
        robot, spoken, clock = FakeRobot(), [], FakeClock(min_step=0.125)  # exact binary steps
        readings = iter([off_by(30.0)] * 30)
        result = wait_for_leader(robot, lambda: next(readings, CATCH), CATCH, events(), 30, spoken.append, clock)
        self.assertEqual(result, "ready")
        self.assertEqual(len(robot.sent), 30)
        self.assertTrue(all({k: a[k] for k in ARM_KEYS} == CATCH for a in robot.sent))
        self.assertEqual(len(spoken), 2)  # 30 frames of 0.125 s span 3.625 s: hints at 0 s and 3 s
        self.assertIn("elbow flex off by 30 degrees", spoken[0])

    def test_timeout(self):
        robot, spoken = FakeRobot(), []
        limits = GateLimits(timeout_s=2.0)
        result = wait_for_leader(robot, lambda: off_by(20.0), CATCH, events(), 30, spoken.append, FakeClock(), limits)
        self.assertEqual(result, "timeout")

    def test_unreadable_leader_waits_and_is_announced(self):
        robot, spoken = FakeRobot(), []
        limits = GateLimits(timeout_s=1.0)
        self.assertEqual(wait_for_leader(robot, lambda: None, CATCH, events(), 30, spoken.append, FakeClock(), limits),
                         "timeout")
        self.assertEqual(spoken, ["Leader arm not readable"])

    def test_stop(self):
        result = wait_for_leader(FakeRobot(), lambda: off_by(20.0), CATCH, events(stop=True), 30, print, FakeClock())
        self.assertEqual(result, "stop")


class RunGateTests(unittest.TestCase):
    def test_moves_then_waits(self):
        robot = FakeRobot(arm=START)
        self.assertEqual(run_gate(robot, lambda: CATCH, CATCH, events(), 30, print, FakeClock()), "ready")
        self.assertEqual({k: robot.sent[-1][k] for k in ARM_KEYS}, CATCH)

    def test_stop_during_the_move(self):
        self.assertEqual(run_gate(FakeRobot(arm=START), lambda: CATCH, CATCH, events(stop=True), 30, print,
                                  FakeClock()), "stop")

    def test_zero_base_holds_the_observed_arm(self):
        robot = FakeRobot(arm=START)
        zero_base(robot)
        self.assertEqual(robot.sent, [{**START, "x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}])


if __name__ == "__main__":
    unittest.main()
