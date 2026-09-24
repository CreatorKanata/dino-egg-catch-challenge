"""tests/robot/test_align.py: Hardware-free checks of the Auto Catch base alignment controller.

align.py is pure and stdlib-only, so the sign conventions (egg right of target -> move right,
-y; egg smaller than target -> forward, +x), the linear speed taper (half error -> half speed,
beyond the full-speed error -> max), zero inside a tolerance and no stall just outside it, the
min-speed deadband, EMA smoothing, the per-frame rate limit, the done / lost / timeout paths,
and the trace row are verified directly with duck-typed eggs.
"""

from dataclasses import dataclass, replace
import unittest

from robot import config
from robot.align import (
    AlignGains,
    AlignState,
    AlignTarget,
    Measurement,
    align_command,
    align_step,
    rate_limit,
    smooth,
    start_align,
    trace_row,
    zero_base,
)

TARGET = AlignTarget(cx=0.5, cy=0.5, h=0.6)
GAINS = AlignGains(max_xy=0.06, full_speed_error_cx=0.2, full_speed_error_h=0.3, min_xy=0.015, tol_cx=0.05,
                   tol_h=0.08, smoothing=0.5, max_accel=0.15, done_frames=3, lost_frames=2, timeout_s=10.0)
FAST = replace(GAINS, max_accel=1000.0, smoothing=1.0)  # no ramp, no smoothing: pure taper checks
FRAME = 1 / 30


@dataclass(frozen=True)
class Egg:
    cx: float
    h: float


ON_TARGET = Egg(cx=0.5, h=0.6)


class TaperTests(unittest.TestCase):
    def test_signs_for_each_error_direction(self):
        cases = ((Egg(0.7, 0.6), "y.vel", -1), (Egg(0.3, 0.6), "y.vel", +1),
                 (Egg(0.5, 0.3), "x.vel", +1), (Egg(0.5, 0.9), "x.vel", -1))
        for egg, axis, sign in cases:
            with self.subTest(egg=egg):
                base, done = align_command(egg, TARGET, GAINS)
                self.assertEqual(base[axis] > 0, sign > 0)
                self.assertNotEqual(base[axis], 0.0)
                self.assertEqual((base["theta.vel"], done), (0.0, False))

    def test_half_error_gives_half_speed_and_beyond_full_error_gives_max(self):
        half, _ = align_command(Egg(0.5 + 0.1, 0.6 - 0.15), TARGET, GAINS)
        self.assertAlmostEqual(half["y.vel"], -0.03)
        self.assertAlmostEqual(half["x.vel"], 0.03)
        full, _ = align_command(Egg(1.0, 0.0), TARGET, GAINS)
        self.assertEqual((full["x.vel"], full["y.vel"]), (0.06, -0.06))
        self.assertEqual(align_command(Egg(0.0, 1.0), TARGET, GAINS)[0]["y.vel"], 0.06)

    def test_inside_tolerance_axis_is_zero_and_edge_does_not_stall(self):
        inside, _ = align_command(Egg(0.5 + 0.049, 0.6 - 0.2), TARGET, GAINS)
        self.assertEqual(inside["y.vel"], 0.0)
        self.assertGreater(inside["x.vel"], 0.0)
        inside_h, _ = align_command(Egg(0.5 + 0.2, 0.6 + 0.079), TARGET, GAINS)
        self.assertEqual(inside_h["x.vel"], 0.0)
        self.assertLess(inside_h["y.vel"], 0.0)
        just_out, done = align_command(Egg(0.5 + 0.051, 0.6 - 0.081), TARGET, GAINS)
        self.assertFalse(done)
        self.assertLessEqual(just_out["y.vel"], -0.015)
        self.assertGreaterEqual(just_out["x.vel"], 0.015)
        self.assertTrue(align_command(Egg(0.5 + 0.049, 0.6 - 0.079), TARGET, GAINS)[1])

    def test_deadband_with_inconsistent_gains(self):
        loose = replace(GAINS, tol_cx=0.01, tol_h=0.01)
        base, done = align_command(Egg(0.5 + 0.03, 0.6 - 0.05), TARGET, loose)  # 0.009 and 0.01 m/s
        self.assertEqual((base, done), (zero_base(), False))

    def test_no_detection_is_zero_and_not_done(self):
        self.assertEqual(align_command(None, TARGET, GAINS), (zero_base(), False))

    def test_config_guards_hold(self):
        for tol, full in ((config.ALIGN_TOL_CX, config.ALIGN_FULL_SPEED_ERROR_CX),
                          (config.ALIGN_TOL_H, config.ALIGN_FULL_SPEED_ERROR_H)):
            self.assertGreaterEqual(config.ALIGN_MAX_XY * tol / full, config.ALIGN_MIN_XY)
        self.assertLessEqual(config.ALIGN_MAX_XY, config.SPEED_LEVELS[0].xy)
        edge = AlignTarget()
        egg = Egg(edge.cx + config.ALIGN_TOL_CX + 0.001, edge.h - config.ALIGN_TOL_H - 0.001)
        base, _ = align_command(egg)
        self.assertNotEqual(base["x.vel"], 0.0)
        self.assertNotEqual(base["y.vel"], 0.0)


class SmoothingAndRateTests(unittest.TestCase):
    def test_smoothing(self):
        self.assertEqual(smooth(None, Egg(0.8, 0.4), 0.5), Measurement(0.8, 0.4))
        averaged = smooth(Measurement(0.4, 0.2), Egg(0.8, 0.4), 0.5)
        self.assertAlmostEqual(averaged.cx, 0.6)
        self.assertAlmostEqual(averaged.h, 0.3)
        self.assertEqual(smooth(Measurement(0.4, 0.2), None, 0.5), Measurement(0.4, 0.2))

    def test_rate_limit_per_frame_and_dt_clamp(self):
        self.assertAlmostEqual(rate_limit(0.0, 0.06, FRAME, 0.15), 0.005)
        self.assertAlmostEqual(rate_limit(0.05, 0.0, FRAME, 0.15), 0.045)
        self.assertEqual(rate_limit(0.01, 0.012, FRAME, 0.15), 0.012)  # reachable within one step
        self.assertAlmostEqual(rate_limit(0.0, 0.06, 5.0, 0.15), 0.15 * 2 / config.LOOP_HZ)
        self.assertEqual(rate_limit(0.02, 0.06, -1.0, 0.15), 0.02)

    def test_step_ramps_up_and_uses_smoothed_egg(self):
        state, base, _ = align_step(start_align(0.0), Egg(0.9, 0.6), 0.0, TARGET, GAINS)
        self.assertEqual(base, zero_base())  # dt = 0 on the start frame
        state, base, _ = align_step(state, Egg(0.9, 0.6), FRAME, TARGET, GAINS)
        self.assertAlmostEqual(base["y.vel"], -0.005)
        self.assertEqual((state.last_y, state.smoothed), (base["y.vel"], Measurement(0.9, 0.6)))
        state, _, _ = align_step(state, Egg(0.5, 0.6), 2 * FRAME, TARGET, GAINS)
        self.assertAlmostEqual(state.smoothed.cx, 0.7)  # EMA, not the raw on-target sample
        self.assertEqual(state.ok_frames, 0)

    def test_flicker_keeps_steering_on_the_smoothed_egg(self):
        patient = replace(GAINS, lost_frames=5)
        moving = replace(start_align(0.0), smoothed=Measurement(0.9, 0.6), last_y=-0.05, updated_at=0.0)
        state, base, result = align_step(moving, None, FRAME, TARGET, patient)
        self.assertEqual((result, state.lost_frames, state.smoothed), ("running", 1, Measurement(0.9, 0.6)))
        self.assertAlmostEqual(base["y.vel"], -0.055)  # still accelerating toward the -0.06 target
        state, base, _ = align_step(state, None, 2 * FRAME, TARGET, patient)
        self.assertAlmostEqual(base["y.vel"], -0.06)
        state, base, _ = align_step(state, Egg(0.9, 0.6), 3 * FRAME, TARGET, patient)
        self.assertEqual((state.lost_frames, base["y.vel"] < 0), (0, True))

    def test_flicker_does_not_reset_done_progress(self):
        gains = replace(FAST, done_frames=4, lost_frames=5)
        state = start_align(0.0)
        for frame, egg in enumerate((ON_TARGET, ON_TARGET, None, None, ON_TARGET)):
            state, _, result = align_step(state, egg, FRAME * frame, TARGET, gains)
            self.assertEqual(result, "running")
        self.assertEqual(state.ok_frames, 3)  # two misses kept the count; nothing counted for them
        self.assertEqual(align_step(state, ON_TARGET, 1.0, TARGET, gains)[2], "done")

    def test_no_egg_yet_means_zero_command(self):
        state, base, _ = align_step(start_align(0.0), None, FRAME, TARGET, GAINS)
        self.assertEqual((base, state.smoothed), (zero_base(), None))


class StepResultTests(unittest.TestCase):
    def test_done_after_consecutive_frames_and_drift_resets(self):
        state = start_align(0.0)
        for frame in range(2):
            state, _, result = align_step(state, ON_TARGET, FRAME * frame, TARGET, FAST)
            self.assertEqual((result, state.ok_frames), ("running", frame + 1))
        state, base, result = align_step(state, Egg(0.8, 0.6), 0.1, TARGET, FAST)
        self.assertEqual((result, state.ok_frames), ("running", 0))
        self.assertLess(base["y.vel"], 0.0)
        for frame in range(2):
            state, _, result = align_step(state, ON_TARGET, 0.2, TARGET, FAST)
        state, base, result = align_step(state, ON_TARGET, 0.3, TARGET, FAST)
        self.assertEqual((state, base, result), (AlignState(), zero_base(), "done"))

    def test_lost_returns_zeros_immediately(self):
        moving = replace(start_align(0.0), last_x=0.05)
        state, _, result = align_step(moving, None, FRAME, TARGET, GAINS)
        self.assertEqual(result, "running")
        self.assertEqual(align_step(state, None, 2 * FRAME, TARGET, GAINS), (AlignState(), zero_base(), "lost"))

    def test_timeout_returns_zeros_immediately(self):
        moving = replace(start_align(0.0), last_x=0.05)
        self.assertEqual(align_step(moving, Egg(0.9, 0.2), 10.0, TARGET, GAINS)[2], "running")
        self.assertEqual(align_step(moving, Egg(0.9, 0.2), 10.01, TARGET, GAINS),
                         (AlignState(), zero_base(), "timeout"))

    def test_config_defaults(self):
        state = start_align(5.0)
        self.assertEqual((state.phase, state.started_at, state.updated_at), ("aligning", 5.0, 5.0))
        for index in range(config.ALIGN_DONE_FRAMES - 1):
            state, _, result = align_step(state, AlignTarget(), 5.0 + FRAME * index)
            self.assertEqual(result, "running")
        self.assertEqual(align_step(state, AlignTarget(), 6.0)[2], "done")
        lost = start_align(0.0)
        for _ in range(config.ALIGN_LOST_FRAMES - 1):
            lost, _, _ = align_step(lost, None, 0.1)
        self.assertEqual(align_step(lost, None, 0.2)[2], "lost")
        self.assertEqual(align_step(start_align(0.0), None, config.ALIGN_TIMEOUT_S + 0.1)[2], "timeout")

    def test_trace_row(self):
        before = replace(start_align(1.0), smoothed=Measurement(0.7, 0.4))
        row = trace_row(before, Egg(0.9, 0.6), 1.5, {"x.vel": 0.01, "y.vel": -0.02, "theta.vel": 0.0}, "running", 0.5)
        self.assertEqual((row.t, row.cx_raw, row.h_raw, row.x_vel, row.y_vel, row.result),
                         (0.5, 0.9, 0.6, 0.01, -0.02, "running"))
        self.assertAlmostEqual(row.cx_smooth, 0.8)
        empty = trace_row(start_align(0.0), None, 0.1, zero_base(), "lost", 0.5)
        self.assertEqual((empty.cx_raw, empty.cx_smooth), (None, None))

    def test_does_not_mutate_input(self):
        state = start_align(0.0)
        align_step(state, ON_TARGET, 0.1, TARGET, GAINS)
        self.assertEqual(state, start_align(0.0))


if __name__ == "__main__":
    unittest.main()
