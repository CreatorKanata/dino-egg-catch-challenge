"""tests/robot/test_align.py: Hardware-free checks of the Auto Catch base alignment controller.

align.py is pure and stdlib-only, so the sign conventions (egg right of target -> move right,
-y; egg smaller than target -> forward, +x), clamping to the slow level, the min-speed deadband,
zero on an axis inside its tolerance (no stall just outside it), and the done / lost / timeout
paths of align_step are verified directly with duck-typed eggs.
"""

from dataclasses import dataclass
import unittest

from robot.align import (
    AlignGains,
    AlignState,
    AlignTarget,
    align_command,
    align_step,
    start_align,
    zero_base,
)
from robot.config import (
    ALIGN_DONE_FRAMES,
    ALIGN_LOST_FRAMES,
    ALIGN_MAX_XY,
    ALIGN_TARGET_CX,
    ALIGN_TARGET_H,
    ALIGN_TIMEOUT_S,
    ALIGN_TOL_CX,
    ALIGN_TOL_H,
)

TARGET = AlignTarget(cx=0.5, cy=0.5, h=0.6)
GAINS = AlignGains(gain_x=0.5, gain_y=0.8, max_xy=0.1, min_xy=0.03, tol_cx=0.04, tol_h=0.06,
                   done_frames=3, lost_frames=2, timeout_s=10.0)


@dataclass(frozen=True)
class Egg:
    cx: float
    h: float


ON_TARGET = Egg(cx=0.5, h=0.6)


class AlignCommandTests(unittest.TestCase):
    def test_signs_for_each_error_direction(self):
        cases = (
            (Egg(0.6, 0.6), "y.vel", -1),  # egg right of target -> move right (-y)
            (Egg(0.4, 0.6), "y.vel", +1),  # egg left -> move left (+y)
            (Egg(0.5, 0.4), "x.vel", +1),  # egg smaller (farther) -> forward
            (Egg(0.5, 0.8), "x.vel", -1),  # egg larger (closer) -> backward
        )
        for egg, axis, sign in cases:
            with self.subTest(egg=egg):
                base, done = align_command(egg, TARGET, GAINS)
                self.assertEqual(base[axis] > 0, sign > 0)
                self.assertNotEqual(base[axis], 0.0)
                self.assertEqual(base["theta.vel"], 0.0)
                self.assertFalse(done)

    def test_proportional_value(self):
        base, _ = align_command(Egg(0.6, 0.5), TARGET, GAINS)
        self.assertAlmostEqual(base["y.vel"], -0.08)
        self.assertAlmostEqual(base["x.vel"], 0.05)

    def test_clamped_to_max_speed(self):
        base, _ = align_command(Egg(1.0, 0.0), TARGET, GAINS)
        self.assertEqual((base["x.vel"], base["y.vel"]), (0.1, -0.1))
        base, _ = align_command(Egg(0.0, 1.0), TARGET, GAINS)
        self.assertEqual((base["x.vel"], base["y.vel"]), (-0.1, 0.1))

    def test_deadband_zeroes_small_commands_outside_tight_tolerances(self):
        tight = AlignGains(gain_x=0.5, gain_y=0.8, max_xy=0.1, min_xy=0.03, tol_cx=0.01, tol_h=0.01)
        base, done = align_command(Egg(0.5 + 0.03, 0.6 - 0.05), TARGET, tight)  # 0.024 and 0.025 m/s
        self.assertEqual((base, done), (zero_base(), False))

    def test_tolerance_edge_no_stall(self):
        just_out = align_command(Egg(0.5 + 0.041, 0.6 - 0.061), TARGET, GAINS)[0]
        self.assertLessEqual(just_out["y.vel"], -0.03)  # 0.8 x 0.041 = 0.0328 m/s, above the minimum
        self.assertGreaterEqual(just_out["x.vel"], 0.03)  # 0.5 x 0.061 = 0.0305 m/s
        inside = align_command(Egg(0.5 + 0.039, 0.6 - 0.2), TARGET, GAINS)[0]
        self.assertEqual(inside["y.vel"], 0.0)  # cx inside its tolerance: no sideways motion
        self.assertGreater(inside["x.vel"], 0.0)  # height still outside: keeps driving forward
        inside_h = align_command(Egg(0.5 + 0.2, 0.6 + 0.059), TARGET, GAINS)[0]
        self.assertEqual(inside_h["x.vel"], 0.0)
        self.assertLess(inside_h["y.vel"], 0.0)

    def test_config_gains_clear_the_deadband_at_the_tolerance_edge(self):
        base, done = align_command(Egg(ALIGN_TARGET_CX + ALIGN_TOL_CX + 0.001, ALIGN_TARGET_H - ALIGN_TOL_H - 0.001))
        self.assertFalse(done)
        self.assertNotEqual(base["y.vel"], 0.0)
        self.assertNotEqual(base["x.vel"], 0.0)

    def test_done_inside_both_tolerances(self):
        self.assertTrue(align_command(ON_TARGET, TARGET, GAINS)[1])
        self.assertTrue(align_command(Egg(0.5 + 0.039, 0.6 - 0.059), TARGET, GAINS)[1])
        self.assertFalse(align_command(Egg(0.5 + 0.05, 0.6), TARGET, GAINS)[1])
        self.assertFalse(align_command(Egg(0.5, 0.6 + 0.07), TARGET, GAINS)[1])

    def test_no_detection_is_zero_and_not_done(self):
        self.assertEqual(align_command(None, TARGET, GAINS), (zero_base(), False))

    def test_config_defaults_cap_at_slow_level(self):
        base, _ = align_command(Egg(1.0, 0.0))
        self.assertEqual(max(abs(base["x.vel"]), abs(base["y.vel"])), ALIGN_MAX_XY)


class AlignStepTests(unittest.TestCase):
    def test_done_after_consecutive_frames_and_drift_resets(self):
        state = start_align(0.0)
        for frame in range(2):
            state, _, result = align_step(state, ON_TARGET, 0.1 * frame, TARGET, GAINS)
            self.assertEqual((result, state.ok_frames), ("running", frame + 1))
        state, base, result = align_step(state, Egg(0.7, 0.6), 0.3, TARGET, GAINS)  # drifted
        self.assertEqual((result, state.ok_frames), ("running", 0))
        self.assertLess(base["y.vel"], 0.0)
        for frame in range(2):
            state, _, result = align_step(state, ON_TARGET, 0.4, TARGET, GAINS)
            self.assertEqual(result, "running")
        state, base, result = align_step(state, ON_TARGET, 0.5, TARGET, GAINS)
        self.assertEqual((state, base, result), (AlignState(), zero_base(), "done"))

    def test_lost_after_consecutive_frames_without_egg(self):
        state, base, result = align_step(start_align(0.0), None, 0.1, TARGET, GAINS)
        self.assertEqual((result, state.lost_frames, base), ("running", 1, zero_base()))
        state, base, result = align_step(state, None, 0.2, TARGET, GAINS)
        self.assertEqual((state, base, result), (AlignState(), zero_base(), "lost"))

    def test_seen_egg_resets_lost_count(self):
        state, _, _ = align_step(start_align(0.0), None, 0.1, TARGET, GAINS)
        state, _, result = align_step(state, Egg(0.8, 0.3), 0.2, TARGET, GAINS)
        self.assertEqual((result, state.lost_frames), ("running", 0))

    def test_timeout_returns_zeros(self):
        moving = AlignState("aligning", 0.0, 0, 0)
        self.assertEqual(align_step(moving, Egg(0.9, 0.2), 10.0, TARGET, GAINS)[2], "running")
        state, base, result = align_step(moving, Egg(0.9, 0.2), 10.01, TARGET, GAINS)
        self.assertEqual((state, base, result), (AlignState(), zero_base(), "timeout"))

    def test_config_defaults(self):
        state = start_align(5.0)
        self.assertEqual((state.phase, state.started_at), ("aligning", 5.0))
        for _ in range(ALIGN_DONE_FRAMES - 1):
            state, _, result = align_step(state, AlignTarget(), 5.1)
            self.assertEqual(result, "running")
        self.assertEqual(align_step(state, AlignTarget(), 5.2)[2], "done")
        lost = start_align(0.0)
        for _ in range(ALIGN_LOST_FRAMES - 1):
            lost, _, _ = align_step(lost, None, 0.1)
        self.assertEqual(align_step(lost, None, 0.2)[2], "lost")
        self.assertEqual(align_step(start_align(0.0), None, ALIGN_TIMEOUT_S + 0.1)[2], "timeout")

    def test_does_not_mutate_input(self):
        state = start_align(0.0)
        align_step(state, ON_TARGET, 0.1, TARGET, GAINS)
        self.assertEqual(state, start_align(0.0))


if __name__ == "__main__":
    unittest.main()
