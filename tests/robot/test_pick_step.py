"""tests/robot/test_pick_step.py: Hardware-free checks of the pick policy step logic (Phase 3 step 3).

pick_step.py is pure, so the per-frame joint cap, the base kept at zero whatever the policy
proposes, the refusal of an incomplete or non-finite proposal, and the stop condition (settling,
closed gripper, minimum time, hard timeout) are verified with fake proposal sequences and a fake
clock. No torch, LeRobot, or robot is involved.
"""

from dataclasses import replace
import unittest

from robot.config import ARM_KEYS, LOOP_HZ
from robot.policy.pick_step import PickLimits, PickState, advance_pick, pick_guard, pick_result, start_pick

FRAME = 1 / LOOP_HZ
GRIPPER = "arm_gripper.pos"
LIMITS = PickLimits(max_s=20.0, min_s=3.0, settle_frames=15, settle_deg=1.0, gripper_closed_max=5.0,
                    max_step_deg_s=90.0)
CATCH = {**{key: 10.0 for key in ARM_KEYS}, GRIPPER: 40.0}
BASE = {"x.vel": 0.3, "y.vel": -0.2, "theta.vel": 45.0}
CLOSED = {**CATCH, GRIPPER: 2.0}


class GuardTests(unittest.TestCase):
    def test_each_arm_key_is_clamped_to_the_step_cap(self):
        proposed = {**CATCH, "arm_elbow_flex.pos": 50.0, "arm_wrist_flex.pos": 10.5, GRIPPER: -20.0, **BASE}
        guarded = pick_guard(CATCH, proposed, FRAME, 90.0)
        self.assertAlmostEqual(guarded["arm_elbow_flex.pos"], 10.0 + 3.0)  # 90 deg/s * 1/30 s
        self.assertAlmostEqual(guarded["arm_wrist_flex.pos"], 10.5)  # inside the cap: taken as is
        self.assertAlmostEqual(guarded[GRIPPER], 40.0 - 3.0)
        self.assertEqual(guarded["arm_shoulder_pan.pos"], 10.0)

    def test_base_is_always_zero(self):
        guarded = pick_guard(CATCH, {**CATCH, **BASE}, FRAME, 90.0)
        self.assertEqual({key: guarded[key] for key in BASE}, {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0})
        self.assertEqual(set(guarded), {*ARM_KEYS, *BASE})

    def test_long_frame_is_clamped_like_playback(self):
        guarded = pick_guard(CATCH, {**CATCH, "arm_elbow_flex.pos": 90.0}, 5.0, 90.0)
        self.assertAlmostEqual(guarded["arm_elbow_flex.pos"], 10.0 + 90.0 * 2 * FRAME)  # dt capped at 2 frames

    def test_missing_or_non_finite_key_raises_a_clear_error(self):
        missing = {key: value for key, value in CATCH.items() if key != GRIPPER}
        with self.assertRaisesRegex(ValueError, "arm_gripper.pos"):
            pick_guard(CATCH, missing, FRAME, 90.0)
        with self.assertRaisesRegex(ValueError, "not finite"):
            pick_guard(CATCH, {**CATCH, "arm_elbow_flex.pos": float("nan")}, FRAME, 90.0)

    def test_inputs_are_not_mutated(self):
        proposed = {**CATCH, **BASE}
        before = dict(proposed)
        pick_guard(CATCH, proposed, FRAME, 90.0)
        self.assertEqual(proposed, before)


class Sequence:
    """Feeds proposals frame by frame and records every result."""

    def __init__(self, now=100.0):
        self.now, self.state, self.results = now, start_pick(now), []

    def feed(self, proposed, commanded=None, frames=1):
        for _ in range(frames):
            self.now += FRAME
            sent = commanded if commanded is not None else proposed
            self.results.append(pick_result(self.state, sent, proposed, self.now, LIMITS))
            self.state = advance_pick(self.state, proposed, 12.5, LIMITS)
        return self.results[-1]


class ResultTests(unittest.TestCase):
    def test_start_state(self):
        self.assertEqual(start_pick(5.0), PickState(started_at=5.0))

    def test_moving_proposals_keep_running(self):
        seq = Sequence()
        for index in range(200):
            result = seq.feed({**CLOSED, "arm_elbow_flex.pos": 10.0 + 2.0 * index})
        self.assertEqual(result, "running")
        self.assertEqual(seq.state.settle_frames, 0)

    def test_settled_closed_gripper_after_min_time_is_done(self):
        seq = Sequence()
        seq.feed(CLOSED, frames=int(LIMITS.min_s / FRAME) - 1)
        self.assertEqual(seq.results[-1], "running")  # settled long ago, but not 3 s yet
        seq.feed(CLOSED, frames=2)
        self.assertEqual(seq.results[-1], "done")
        self.assertEqual(seq.state.last_inference_ms, 12.5)

    def test_needs_settle_frames_in_a_row(self):
        seq = Sequence()
        seq.feed({**CLOSED, "arm_elbow_flex.pos": 0.0}, frames=int(LIMITS.min_s / FRAME) + 5)
        seq.feed(CLOSED)  # a 10 deg jump resets the count
        self.assertEqual(seq.state.settle_frames, 0)
        seq.feed({**CLOSED, "arm_elbow_flex.pos": 10.9}, frames=LIMITS.settle_frames - 1)  # 0.9 deg < 1: settling
        self.assertEqual(seq.results[-1], "running")
        self.assertEqual(seq.feed({**CLOSED, "arm_elbow_flex.pos": 10.9}), "done")

    def test_open_gripper_never_finishes(self):
        seq = Sequence()
        seq.feed(CATCH, frames=int(LIMITS.max_s / FRAME) - 1)
        self.assertEqual(set(seq.results), {"running"})

    def test_gripper_check_uses_the_command_sent(self):
        seq = Sequence()
        seq.feed(CLOSED, commanded=CATCH, frames=int(LIMITS.min_s / FRAME) + 20)
        self.assertEqual(seq.results[-1], "running")  # proposal closed, rate-limited command still open

    def test_timeout_at_the_limit(self):
        state = start_pick(0.0)
        self.assertEqual(pick_result(state, CATCH, CATCH, LIMITS.max_s - 0.01, LIMITS), "running")
        self.assertEqual(pick_result(state, CATCH, CATCH, LIMITS.max_s, LIMITS), "timeout")

    def test_advance_counts_frames_and_keeps_the_arm_proposal(self):
        state = advance_pick(start_pick(0.0), {**CATCH, **BASE}, 3.0, LIMITS)
        self.assertEqual((state.frames, state.settle_frames, state.last_inference_ms), (1, 0, 3.0))
        self.assertEqual(state.previous, CATCH)
        self.assertEqual(replace(state, previous=None).frames, 1)


if __name__ == "__main__":
    unittest.main()
