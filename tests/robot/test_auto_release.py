"""tests/robot/test_auto_release.py: Hardware-free checks of the Auto Release state machine.

auto_release.py is pure, so the phase sequence align -> home -> play -> return_home with fake
basket detections and a fake clock, the gripper held in `home`, the slow approach to the motion's
first frame, the per-frame joint step cap in `play`, the zero base outside `align`, the
progress fraction, and every terminal outcome (lost, alignment timeout, home timeouts) are
verified directly. Stop cancellation is a mode-manager rule (test_mode_manager_release.py).
"""

from dataclasses import dataclass, replace
import unittest

from robot.align import zero_base
from robot.auto_release import (
    GRIPPER_KEY,
    RELEASE_TARGET,
    ReleaseLimits,
    ReleaseState,
    release_progress,
    release_step,
    start_release,
)
from robot.config import ALIGN_DONE_FRAMES, ALIGN_LOST_FRAMES, ALIGN_TIMEOUT_S, ARM_KEYS, LOOP_HZ

FRAME = 1 / LOOP_HZ
LIMITS = ReleaseLimits(approach_speed=30.0, tolerance=3.0, home_timeout_s=8.0, max_joint_step_deg_s=90.0)
START = {**{key: 20.0 for key in ARM_KEYS}, GRIPPER_KEY: 5.0}  # egg held: gripper nearly closed
HOME = {**{key: 0.0 for key in ARM_KEYS}, GRIPPER_KEY: 50.0}  # home has the mouth open
MOTION = tuple({**{key: 0.0 for key in ARM_KEYS}, GRIPPER_KEY: 5.0 + 5.0 * index} for index in range(10))


@dataclass(frozen=True)
class Basket:
    cx: float
    w: float


ON_TARGET = Basket(cx=RELEASE_TARGET.cx, w=RELEASE_TARGET.h)
FAR_LEFT = Basket(cx=0.2, w=0.5)


class Runner:
    """Steps the machine with a fake clock and keeps the commanded pose like the adapter."""

    def __init__(self, commanded=START, frames=MOTION, now=100.0):
        self.now, self.commanded = now, dict(commanded)
        self.state = start_release(now, HOME, frames)
        self.log = []

    def step(self, basket=ON_TARGET, dt=FRAME):
        self.now += dt
        result = release_step(self.state, basket, self.commanded, self.now, LIMITS)
        self.state, self.commanded = result.state, result.arm
        self.log.append(result)
        return result

    def run_until(self, phase, limit=2000, basket=ON_TARGET):
        for _ in range(limit):
            if self.state.phase == phase:
                return
            self.step(basket)
        raise AssertionError(f"never reached {phase}; stuck in {self.state.phase}")


class AlignPhaseTests(unittest.TestCase):
    def test_align_drives_toward_the_basket_and_holds_the_arm(self):
        runner = Runner()
        for _ in range(3):
            result = runner.step(FAR_LEFT)
        self.assertEqual((result.state.phase, result.outcome), ("align", "running"))
        self.assertGreater(result.base["y.vel"], 0.0)  # basket left of the target -> move left
        self.assertGreater(result.base["x.vel"], 0.0)  # basket narrower than the target -> forward
        self.assertEqual(result.arm, START)

    def test_done_moves_on_to_home_with_zeros(self):
        runner = Runner()
        for _ in range(ALIGN_DONE_FRAMES):
            result = runner.step()
        self.assertEqual((result.state.phase, result.base, result.outcome), ("home", zero_base(), "running"))

    def test_lost_and_timeout_end_with_zeros_and_a_held_arm(self):
        runner = Runner()
        for _ in range(ALIGN_LOST_FRAMES):
            result = runner.step(None)
        self.assertEqual((result.outcome, result.state, result.base, result.arm), ("lost", ReleaseState(), zero_base(), START))
        late = release_step(start_release(0.0, HOME, MOTION), FAR_LEFT, START, ALIGN_TIMEOUT_S + 0.1, LIMITS)
        self.assertEqual((late.outcome, late.base), ("align_timeout", zero_base()))


class ArmPhaseTests(unittest.TestCase):
    def setUp(self):
        self.runner = Runner()
        self.runner.run_until("home")

    def test_home_keeps_the_gripper_and_the_base_at_zero(self):
        runner = self.runner
        runner.run_until("play")
        home_frames = [result for result in runner.log if result.state.phase in ("home", "play")]
        self.assertTrue(all(result.arm[GRIPPER_KEY] == START[GRIPPER_KEY] for result in home_frames))
        self.assertTrue(all(result.base == zero_base() for result in home_frames))
        self.assertEqual(runner.log[-1].outcome, "play_started")
        self.assertTrue(all(abs(runner.commanded[key] - HOME[key]) <= 3.0 for key in ARM_KEYS if key != GRIPPER_KEY))

    def test_home_moves_at_most_the_engagement_speed_per_frame(self):
        before = dict(self.runner.commanded)
        after = self.runner.step().arm
        for key in ARM_KEYS:
            self.assertLessEqual(abs(after[key] - before[key]), 30.0 * FRAME + 1e-9)

    def test_play_follows_the_recording_with_the_step_cap_then_returns_home(self):
        runner = self.runner
        runner.run_until("return_home")
        self.assertEqual(runner.log[-1].arm, MOTION[-1])  # last frame (mouth open) reached exactly
        steps = [abs(b.arm[key] - a.arm[key]) for a, b in zip(runner.log, runner.log[1:]) for key in ARM_KEYS]
        self.assertLessEqual(max(steps), 90.0 * FRAME + 1e-9)
        runner.run_until("idle")
        self.assertEqual((runner.log[-1].outcome, runner.log[-1].arm), ("done", HOME))
        self.assertTrue(all(result.base == zero_base() for result in runner.log if result.state.phase != "align"))

    def test_playback_is_capped_even_for_a_jumpy_recording(self):
        jumpy = (MOTION[0], {**MOTION[0], "arm_elbow_flex.pos": 80.0})
        runner = Runner(commanded={**HOME, GRIPPER_KEY: 5.0}, frames=jumpy)
        runner.state = replace(runner.state, phase="play", started_at=runner.now, index=-1)
        values = [runner.step().arm["arm_elbow_flex.pos"] for _ in range(4)]
        self.assertEqual([round(value, 6) for value in values], [0.0, 3.0, 6.0, 9.0])

    def test_first_frame_is_approached_slowly_when_far(self):
        far_start = tuple({**pose, "arm_shoulder_pan.pos": 40.0} for pose in MOTION)
        runner = Runner(commanded={**HOME, GRIPPER_KEY: 5.0}, frames=far_start)
        runner.state = replace(runner.state, phase="play", started_at=runner.now, index=-1)
        first = runner.step()
        self.assertEqual((first.state.index, round(first.arm["arm_shoulder_pan.pos"], 6)), (-1, 1.0))
        self.assertIsNone(release_progress(first.state))
        runner.run_until("return_home")
        self.assertEqual(release_progress(replace(runner.state, phase="play", index=9)), 1.0)

    def test_first_frame_approach_keeps_the_gripper_until_playback(self):
        opened = tuple({**pose, "arm_shoulder_pan.pos": 10.0, GRIPPER_KEY: 40.0} for pose in MOTION[:3])
        runner = Runner(commanded={**HOME, GRIPPER_KEY: 10.0}, frames=opened)
        runner.state = replace(runner.state, phase="play", started_at=runner.now, index=-1)
        approach = []
        while runner.state.phase == "play" and runner.state.index < 0:
            approach.append(runner.step().arm)
        self.assertGreater(len(approach), 5)  # 10 deg at 30 deg/s: several frames of approach
        self.assertTrue(all(pose[GRIPPER_KEY] == 10.0 for pose in approach))
        playing = runner.step().arm  # first real playback frame: now toward the recording, capped
        self.assertAlmostEqual(playing[GRIPPER_KEY], 10.0 + 90.0 * FRAME)

    def test_progress_fraction(self):
        state = replace(start_release(0.0, HOME, MOTION), phase="play", index=3)
        self.assertAlmostEqual(release_progress(state), 3 / 9)
        self.assertIsNone(release_progress(replace(state, phase="home")))


class TimeoutTests(unittest.TestCase):
    def phase_state(self, phase, commanded_far):
        state = replace(start_release(0.0, HOME, MOTION), phase=phase, started_at=0.0, updated_at=0.0)
        commanded = {**HOME, "arm_elbow_flex.pos": 179.0} if commanded_far else HOME
        return state, commanded

    def test_home_and_return_home_time_out_with_a_held_arm(self):
        for phase in ("home", "return_home"):
            with self.subTest(phase=phase):
                state, commanded = self.phase_state(phase, True)
                result = release_step(state, None, commanded, LIMITS.home_timeout_s + 0.01, LIMITS)
                self.assertEqual((result.outcome, result.arm, result.base), ("home_timeout", commanded, zero_base()))

    def test_first_frame_approach_times_out(self):
        state, commanded = self.phase_state("play", True)
        result = release_step(state, None, commanded, LIMITS.home_timeout_s + 0.01, LIMITS)
        self.assertEqual(result.outcome, "start_timeout")

    def test_idle_state_holds_and_does_not_mutate(self):
        idle = ReleaseState()
        result = release_step(idle, ON_TARGET, START, 1.0, LIMITS)
        self.assertEqual((result.state, result.arm, result.base), (idle, START, zero_base()))
        self.assertIsNot(result.arm, START)


if __name__ == "__main__":
    unittest.main()
