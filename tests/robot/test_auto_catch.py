"""tests/robot/test_auto_catch.py: Hardware-free checks of the Auto Catch state machine (Phase 3 step 1).

auto_catch.py is pure, so the phase sequence align -> to_catch -> wrist_check -> pick_stub ->
to_release with fake egg and wrist detections and a fake clock, the gripper following the catch
pose in `to_catch` and held in `to_release`, the per-frame rate limit, the zero base outside
`align`, the wrist check (disabled, enabled with and without an egg), and every terminal outcome
(lost, alignment timeout, pose timeouts, done) are verified directly. Stop cancellation is a
mode-manager rule (test_mode_manager_catch.py).
"""

from dataclasses import dataclass, replace
import unittest

from robot.align import zero_base
from robot.auto_catch import CATCH_PHASES, CatchLimits, CatchState, catch_step, start_catch
from robot.auto_release import GRIPPER_KEY
from robot.config import ALIGN_DONE_FRAMES, ALIGN_LOST_FRAMES, ALIGN_TARGET_CX, ALIGN_TARGET_H, ALIGN_TIMEOUT_S
from robot.config import ARM_KEYS, LOOP_HZ

FRAME = 1 / LOOP_HZ
LIMITS = CatchLimits(approach_speed=30.0, tolerance=3.0, pose_timeout_s=8.0, wrist_check_frames=10,
                     wrist_check_enabled=False)
ENABLED = replace(LIMITS, wrist_check_enabled=True)
START = {**{key: 0.0 for key in ARM_KEYS}, GRIPPER_KEY: 5.0}  # release pose, mouth closed
CATCH = {**{key: 12.0 for key in ARM_KEYS}, GRIPPER_KEY: 40.0}  # head down, mouth open as recorded
HOME = {**{key: 0.0 for key in ARM_KEYS}, GRIPPER_KEY: 50.0}  # the release pose's own gripper value


@dataclass(frozen=True)
class Egg:
    cx: float
    h: float


ON_TARGET = Egg(cx=ALIGN_TARGET_CX, h=ALIGN_TARGET_H)
FAR_RIGHT = Egg(cx=0.8, h=0.4)
WRIST_EGG = object()  # any non-None wrist detection counts as "egg in view"


class Runner:
    """Steps the machine with a fake clock and keeps the commanded pose like the adapter."""

    def __init__(self, limits=LIMITS, commanded=START, now=100.0):
        self.now, self.commanded, self.limits = now, dict(commanded), limits
        self.state = start_catch(now, CATCH, HOME)
        self.log = []

    def step(self, egg=ON_TARGET, wrist=None, dt=FRAME):
        self.now += dt
        result = catch_step(self.state, egg, wrist, self.commanded, self.now, self.limits)
        self.state, self.commanded = result.state, result.arm
        self.log.append(result)
        return result

    def run_until(self, phase, limit=2000, wrist=None):
        for _ in range(limit):
            if self.state.phase == phase:
                return
            self.step(wrist=wrist)
        raise AssertionError(f"never reached {phase}; stuck in {self.state.phase}")

    def outcomes(self):
        return [result.outcome for result in self.log if result.outcome != "running"]


class AlignPhaseTests(unittest.TestCase):
    def test_align_drives_the_base_and_holds_the_arm(self):
        runner = Runner()
        for _ in range(3):
            result = runner.step(FAR_RIGHT)
        self.assertEqual((result.state.phase, result.outcome, result.align_result), ("align", "running", "running"))
        self.assertLess(result.base["y.vel"], 0.0)  # egg right of the target -> move right
        self.assertGreater(result.base["x.vel"], 0.0)  # egg smaller than the target -> forward
        self.assertEqual(result.arm, START)

    def test_done_moves_on_to_the_catch_pose_with_zeros(self):
        runner = Runner()
        for _ in range(ALIGN_DONE_FRAMES):
            result = runner.step()
        self.assertEqual((result.state.phase, result.base, result.align_result), ("to_catch", zero_base(), "done"))

    def test_lost_and_timeout_end_with_zeros_and_a_held_arm(self):
        runner = Runner()
        for _ in range(ALIGN_LOST_FRAMES):
            result = runner.step(None)
        self.assertEqual((result.outcome, result.state, result.base, result.arm), ("lost", CatchState(), zero_base(), START))
        late = catch_step(start_catch(0.0, CATCH, HOME), FAR_RIGHT, None, START, ALIGN_TIMEOUT_S + 0.1, LIMITS)
        self.assertEqual((late.outcome, late.base), ("align_timeout", zero_base()))


class ArmPhaseTests(unittest.TestCase):
    def test_to_catch_follows_the_recorded_gripper_at_the_engagement_speed(self):
        runner = Runner()
        runner.run_until("to_catch")
        before = dict(runner.commanded)
        after = runner.step().arm
        for key in ARM_KEYS:
            self.assertLessEqual(abs(after[key] - before[key]), 30.0 * FRAME + 1e-9)
        self.assertGreater(after[GRIPPER_KEY], before[GRIPPER_KEY])  # opening toward the recorded 40
        runner.run_until("wrist_check")
        self.assertTrue(all(abs(runner.commanded[key] - CATCH[key]) <= 3.0 for key in ARM_KEYS))
        self.assertTrue(all(result.base == zero_base() for result in runner.log if result.state.phase != "align"))

    def test_disabled_wrist_check_holds_then_runs_the_stub_and_returns_to_the_release_pose(self):
        runner = Runner()
        runner.run_until("wrist_check")
        held = dict(runner.commanded)
        checks = [runner.step() for _ in range(LIMITS.wrist_check_frames)]
        self.assertTrue(all(result.arm == held for result in checks))  # held for the whole check
        self.assertEqual(checks[-1].state.phase, "pick_stub")
        stub = runner.step()
        self.assertEqual((stub.outcome, stub.state.phase, stub.arm), ("policy_stub", "to_release", held))
        runner.run_until("idle")
        self.assertEqual(runner.outcomes(), ["policy_stub", "done"])
        final = runner.log[-1].arm
        self.assertEqual(final[GRIPPER_KEY], held[GRIPPER_KEY])  # gripper kept at its current value
        self.assertTrue(all(abs(final[key] - HOME[key]) <= 3.0 for key in ARM_KEYS if key != GRIPPER_KEY))

    def test_enabled_check_without_an_egg_returns_to_the_release_pose(self):
        runner = Runner(limits=ENABLED)
        runner.run_until("wrist_check")
        for _ in range(ENABLED.wrist_check_frames):
            result = runner.step(wrist=None)
        self.assertEqual((result.outcome, result.state.phase), ("no_wrist_egg", "to_release"))
        runner.run_until("idle")
        self.assertEqual(runner.outcomes(), ["no_wrist_egg", "done"])

    def test_enabled_check_with_an_egg_in_any_frame_goes_to_the_stub(self):
        runner = Runner(limits=ENABLED)
        runner.run_until("wrist_check")
        seen = [None] * (ENABLED.wrist_check_frames - 1) + [WRIST_EGG]
        for wrist in reversed(seen):  # the egg is seen only in the first check frame
            result = runner.step(wrist=wrist)
        self.assertEqual((result.outcome, result.state.phase), ("running", "pick_stub"))

    def test_rate_limit_holds_for_every_arm_phase(self):
        runner = Runner()
        runner.run_until("idle")
        steps = [abs(b.arm[key] - a.arm[key]) for a, b in zip(runner.log, runner.log[1:]) for key in ARM_KEYS]
        self.assertLessEqual(max(steps), 30.0 * FRAME + 1e-9)


class TimeoutTests(unittest.TestCase):
    def phase_state(self, phase):
        return replace(start_catch(0.0, CATCH, HOME), phase=phase, started_at=0.0, updated_at=0.0)

    def test_catch_pose_timeout_warns_and_returns_to_the_release_pose(self):
        far = {**START, "arm_elbow_flex.pos": 179.0}
        result = catch_step(self.phase_state("to_catch"), None, None, far, LIMITS.pose_timeout_s + 0.01, LIMITS)
        self.assertEqual((result.outcome, result.state.phase, result.arm, result.base),
                         ("catch_timeout", "to_release", far, zero_base()))

    def test_release_pose_timeout_ends_the_action_with_a_held_arm(self):
        far = {**START, "arm_elbow_flex.pos": 179.0}
        result = catch_step(self.phase_state("to_release"), None, None, far, LIMITS.pose_timeout_s + 0.01, LIMITS)
        self.assertEqual((result.outcome, result.state, result.arm, result.base),
                         ("release_timeout", CatchState(), far, zero_base()))

    def test_idle_state_holds_and_does_not_mutate(self):
        idle = CatchState()
        result = catch_step(idle, ON_TARGET, WRIST_EGG, START, 1.0, LIMITS)
        self.assertEqual((result.state, result.arm, result.base, result.outcome), (idle, START, zero_base(), "running"))
        self.assertIsNot(result.arm, START)

    def test_phase_names(self):
        self.assertEqual(CATCH_PHASES, ("idle", "align", "to_catch", "wrist_check", "pick_stub", "to_release"))


if __name__ == "__main__":
    unittest.main()
