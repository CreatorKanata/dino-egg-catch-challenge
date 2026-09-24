"""tests/robot/test_auto_catch_pick.py: Hardware-free checks of Auto Catch's `pick` phase (Phase 3 step 3).

auto_catch.py calls the pick policy through an injected callable, so a fake `policy_act` drives the
phase: the raw observation reaching it, every arm key capped per frame with the base at zero, the
stop condition ending with "pick_done" and then "done", the timeout and a runner exception (and an
incomplete action) warning and returning to the release pose (their warning repeated once back),
the stub without a runner, and holding while no observation exists yet. Stop cancelling the pick is
checked through the mode manager (step_auto_catch never calls the runner afterwards).
"""

from dataclasses import replace
import unittest

from robot.align import zero_base
from robot.auto_catch import CatchLimits, CatchState, catch_step
from robot.auto_release import GRIPPER_KEY
from robot.config import ARM_KEYS, LOOP_HZ
from robot.manual_mode import CommandResult, step_auto_catch
from robot.mode_manager import AppState, apply_command
from robot.policy.pick_step import PickLimits, start_pick

FRAME = 1 / LOOP_HZ
PICK = PickLimits(max_s=2.0, min_s=0.5, settle_frames=5, settle_deg=1.0, gripper_closed_max=5.0, max_step_deg_s=90.0)
LIMITS = CatchLimits(approach_speed=30.0, tolerance=3.0, pose_timeout_s=8.0, wrist_check_enabled=False, pick=PICK)
CATCH = {**{key: 12.0 for key in ARM_KEYS}, GRIPPER_KEY: 40.0}
HOME = {**{key: 0.0 for key in ARM_KEYS}, GRIPPER_KEY: 50.0}
RAW = {"wrist": "raw RGB frame", "front": "raw RGB frame"}  # the loop passes LeKiwiClient's dict as is


class FakePolicy:
    """Returns the queued proposals in turn (the last one repeats), or raises `error`."""

    def __init__(self, *proposals, error=None):
        self.proposals, self.error, self.seen = list(proposals), error, []

    def __call__(self, observation):
        self.seen.append(observation)
        if self.error is not None:
            raise self.error
        return self.proposals.pop(0) if len(self.proposals) > 1 else self.proposals[0]


def picking(now=100.0):
    return CatchState(phase="pick", started_at=now, catch=dict(CATCH), home=dict(HOME), updated_at=now,
                      pick=start_pick(now))


class Runner:
    def __init__(self, policy, observation=RAW):
        self.now, self.state, self.commanded, self.log = 100.0, picking(), dict(CATCH), []
        self.policy, self.observation = policy, observation

    def step(self):
        self.now += FRAME
        result = catch_step(self.state, None, None, self.commanded, self.now, LIMITS, self.observation, self.policy)
        self.state, self.commanded = result.state, result.arm
        self.log.append(result)
        return result

    def run(self, limit=600):
        for _ in range(limit):
            if self.step().state.phase == "idle":
                break
        return [result.outcome for result in self.log if result.outcome != "running"]


class PickPhaseTests(unittest.TestCase):
    def test_raw_observation_reaches_the_runner_and_moves_are_capped(self):
        policy = FakePolicy({**CATCH, "arm_elbow_flex.pos": 60.0, "x.vel": 0.3, "theta.vel": 90.0})
        result = Runner(policy).step()
        self.assertIs(policy.seen[0], RAW)
        self.assertEqual((result.state.phase, result.outcome, result.base), ("pick", "running", zero_base()))
        self.assertAlmostEqual(result.arm["arm_elbow_flex.pos"], 12.0 + 90.0 * FRAME)
        self.assertEqual(set(result.arm), set(ARM_KEYS))  # only the six arm keys go to the robot
        self.assertEqual(result.state.pick.frames, 1)

    def test_settled_closed_mouth_ends_with_caught_then_ready(self):
        closed = {**CATCH, GRIPPER_KEY: 2.0}
        runner = Runner(FakePolicy(closed))
        outcomes = runner.run()
        self.assertEqual(outcomes, ["pick_done", "done"])
        done_at = next(index for index, result in enumerate(runner.log) if result.outcome == "pick_done")
        self.assertGreaterEqual(done_at + 1, PICK.min_s / FRAME - 1)  # not before the minimum time
        final = runner.log[-1].arm
        self.assertAlmostEqual(final[GRIPPER_KEY], 2.0)  # the egg stays held on the way back
        self.assertTrue(all(result.base == zero_base() for result in runner.log))

    def test_open_mouth_times_out_and_returns(self):
        runner = Runner(FakePolicy(CATCH))
        self.assertEqual(runner.run(), ["pick_timeout", "pick_timeout_returned"])
        self.assertLessEqual(len(runner.policy.seen), PICK.max_s / FRAME + 1)

    def test_runner_error_and_bad_action_return_with_a_warning(self):
        for policy in (FakePolicy(error=RuntimeError("mps out of memory")),
                       FakePolicy({key: value for key, value in CATCH.items() if key != GRIPPER_KEY})):
            with self.subTest(policy=policy.error), self.assertLogs("robot.auto_catch", "ERROR") as logs:
                runner = Runner(policy)
                first = runner.step()
                self.assertEqual((first.outcome, first.state.phase, first.arm), ("policy_error", "to_release", CATCH))
                self.assertEqual(runner.run(), ["policy_error", "policy_error_returned"])
            self.assertEqual(len(logs.output), 1)  # logged once, with the traceback
            self.assertIn("Traceback", logs.output[0])

    def test_no_runner_is_the_stub(self):
        runner = Runner(None)
        stub = runner.step()
        self.assertEqual((stub.outcome, stub.state.phase, stub.arm), ("policy_stub", "to_release", CATCH))
        self.assertEqual(runner.run(), ["policy_stub", "done"])

    def test_no_observation_yet_holds(self):
        policy = FakePolicy(CATCH)
        result = Runner(policy, observation=None).step()
        self.assertEqual((result.state.phase, result.arm, policy.seen), ("pick", CATCH, []))

    def test_stop_cancels_the_pick_and_the_runner_is_not_called_again(self):
        policy = FakePolicy(CATCH)
        running = replace(AppState(), action="auto_catch", catch=picking())
        stopped = apply_command(running, "stop", 100.1)
        self.assertEqual((stopped.state.action, stopped.state.catch), ("none", CatchState()))
        result, base, arm, _ = step_auto_catch(CommandResult(stopped.state, True, True), None, None, CATCH, 100.2,
                                               LIMITS, RAW, policy)
        self.assertEqual((base, arm, policy.seen), (None, None, []))


if __name__ == "__main__":
    unittest.main()
