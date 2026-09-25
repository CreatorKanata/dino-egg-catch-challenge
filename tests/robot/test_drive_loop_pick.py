"""tests/robot/test_drive_loop_pick.py: Loop wiring of the Auto Catch pick policy (Phase 3 step 3).

Uses the fake devices in loop_fakes.py (the adapter returns RGB-ordered Pi frames like
LeKiwiClient), an egg at the target, and a fake pick runner. Verified without hardware, torch, or
OpenCV: no runner -> the stub; with a runner the raw RGB observation reaches it while the
signboard still gets the BGR copies, the runner is reset once per pick, its capped arm action is
sent with the base at zero, "Catch finished" then "Ready"; Stop during the pick zeros the base, holds the
arm, and never calls the runner again; a failing reset becomes "Policy error"; the front-camera
detectors are skipped in the arm-only phases (pick included) but run while aligning; the startup
loader returns None (stub) when disabled or when loading fails; and the ensembling flags parse.
"""

import unittest
from unittest import mock

import numpy as np

try:
    from robot import drive_loop
    from robot.drive_loop import DriveDevices, step
except ImportError:  # pragma: no cover - depends on the environment
    DriveDevices = None

from loop_fakes import CATCH_POSE, DRIVING, FORWARD, FRONT_BGR, FRONT_RGB, NEAR, ZEROS, FakeAdapter, FakeLeader
from loop_fakes import FakeReader, FakeView, temp_arm_paths
from robot.auto_catch import CatchLimits
from robot.config import ALIGN_TARGET_CX, ALIGN_TARGET_H, ALIGN_TARGET_W_EGG, LOOP_HZ
from robot.policy.pick_step import PickLimits
from robot.policy.config_policy import PICK_TEMPORAL_ENSEMBLE_COEFF
from robot.teleop_drive import load_pick_policy, parse_args
from robot.vision.egg_size import EggDetection

ON_TARGET = EggDetection(cx=ALIGN_TARGET_CX, cy=0.5, w=ALIGN_TARGET_W_EGG, h=ALIGN_TARGET_H, color="green", spots=3,
                         area_px=20000)
GRIPPER = "arm_gripper.pos"
LIMITS = CatchLimits(wrist_check_enabled=False, pick=PickLimits(max_s=3.0, min_s=0.5, settle_frames=5))
CLOSED = {**CATCH_POSE, GRIPPER: 0.0, "x.vel": 0.2, "theta.vel": 30.0}  # the base must stay at zero anyway


class WristAdapter(FakeAdapter):
    def observe(self):
        return {**super().observe(), "wrist": FRONT_RGB}


class FakeRunner:
    def __init__(self, proposal=CLOSED, reset_error=None):
        self.proposal, self.reset_error, self.seen, self.resets = proposal, reset_error, [], 0

    def reset(self):
        self.resets += 1
        if self.reset_error is not None:
            raise self.reset_error

    def act(self, observation):
        self.seen.append(observation)
        return dict(self.proposal)


class PickLoopTests(unittest.TestCase):
    def setUp(self):
        if DriveDevices is None:
            self.skipTest("LeRobot is not installed")
        for name, value in (("detect_eggs", (ON_TARGET,)), ("detect_basket", None), ("append_row", None)):
            patcher = mock.patch.object(drive_loop, name, return_value=value)
            setattr(self, name, patcher.start())
            self.addCleanup(patcher.stop)
        self.clock = 100.0
        clock = mock.patch.object(drive_loop.time, "monotonic", side_effect=self.tick)
        clock.start()
        self.addCleanup(clock.stop)
        self.view, self.paths = FakeView(keep_running=True), temp_arm_paths(self)

    def devices(self, runner):
        return DriveDevices(reader=FakeReader(FORWARD), adapter=WristAdapter(), camera=None, view=self.view,
                            use_rerun=False, leader=FakeLeader(NEAR), arm_paths=self.paths, catch_limits=LIMITS,
                            pick_policy=runner)

    def tick(self):
        self.clock += 1 / LOOP_HZ
        return self.clock

    def run_catch(self, parts, stop_after_pick_frames=None, limit=900):
        self.view.commands = ()
        state, _, _ = step(parts, DRIVING, self.clock)
        self.view.commands = ("hi",)
        state, _, _ = step(parts, state, self.clock)
        notices, pick_frames, self.detections = [], 0, []
        for _ in range(limit):
            calls = self.detect_eggs.call_count
            if state.app.catch.phase == "pick":
                pick_frames += 1
                if pick_frames == stop_after_pick_frames:
                    self.view.commands = ("stop",)
            state, _, _ = step(parts, state, self.clock)
            self.view.commands = ()
            notices.append(state.app.notice)
            # phase after the step: detection follows the phase the frame ends in
            self.detections.append((state.app.catch.phase, self.detect_eggs.call_count - calls,
                                    self.view.rendered[-1][2].overlays))
            if state.app.action == "none":
                break
        return state, notices

    def test_without_a_runner_the_stub_runs(self):
        parts = self.devices(None)
        state, notices = self.run_catch(parts)
        self.assertIn("Catch: policy not available yet", notices)
        self.assertEqual(state.app.notice, "Ready")

    def test_raw_rgb_frames_reach_the_runner_and_the_display_gets_bgr(self):
        runner = FakeRunner()
        parts = self.devices(runner)
        state, notices = self.run_catch(parts)
        self.assertTrue(runner.seen)
        np.testing.assert_array_equal(runner.seen[0]["wrist"], FRONT_RGB)  # RGB, as LeKiwiClient delivers it
        np.testing.assert_array_equal(runner.seen[0]["front"], FRONT_RGB)
        np.testing.assert_array_equal(self.view.rendered[-1][0]["wrist"], FRONT_BGR)  # the signboard: BGR
        self.assertEqual(runner.resets, 1)
        self.assertIn("Catch finished", notices)
        self.assertNotIn("Caught!", notices)  # reserved for a verified success (not implemented)
        self.assertNotIn("Catch: policy not available yet", notices)
        self.assertEqual(state.app.notice, "Ready")
        self.assertTrue(any(arm[GRIPPER] < CATCH_POSE[GRIPPER] - 1 for arm in parts.adapter.arms))  # mouth closed
        self.assertAlmostEqual(parts.adapter.arms[-1][GRIPPER], 0.0, delta=3.0)  # held on the way back

    def test_pick_frames_send_zero_base_and_show_the_pick_time(self):
        runner = FakeRunner(proposal={**CATCH_POSE, "x.vel": 0.3})
        parts = self.devices(runner)
        self.run_catch(parts, stop_after_pick_frames=12)
        self.assertEqual(len(runner.seen), 11)  # one call per pick frame before the Stop frame, none on it
        self.assertTrue(all(sent == ZEROS for sent in parts.adapter.sent[-12:]))
        shown = [status.phase_s for _, _, status in self.view.rendered if status.phase == "pick"]
        self.assertTrue(shown and all(seconds is not None and seconds >= 0 for seconds in shown))

    def test_stop_during_pick_zeros_holds_and_never_calls_again(self):
        runner = FakeRunner(proposal={**CATCH_POSE, GRIPPER: 0.0})
        parts = self.devices(runner)
        state, _ = self.run_catch(parts, stop_after_pick_frames=6)
        calls, held = len(runner.seen), parts.adapter.arms[-1]
        self.assertEqual((state.app.action, state.app.stopped), ("none", True))
        state, _, _ = step(parts, state, self.clock)
        self.assertEqual((len(runner.seen), parts.adapter.sent[-1], parts.adapter.arms[-1]), (calls, ZEROS, held))

    def test_detectors_are_skipped_in_arm_only_phases(self):
        self.run_catch(self.devices(FakeRunner()))
        counts = {}
        for phase, calls, _ in self.detections:
            counts.setdefault(phase, set()).add(calls)
        self.assertEqual(counts["align"], {1})  # the alignment still reads the egg every frame
        self.assertEqual(counts["wrist_check"], {1})  # not an arm-only phase in the list
        for phase in ("to_catch", "pick", "to_release"):
            self.assertEqual(counts[phase], {0}, phase)
        self.assertEqual(self.detect_basket.call_count, self.detect_eggs.call_count)
        kinds = {overlay.kind for phase, _, overlays in self.detections if phase == "pick" for overlay in overlays}
        self.assertNotIn("egg_ok", kinds)  # no stale egg outline while skipped

    def test_failing_reset_is_a_policy_error(self):
        runner = FakeRunner(reset_error=RuntimeError("queue"))
        parts = self.devices(runner)
        with self.assertLogs("robot.auto_catch", "ERROR"):
            state, notices = self.run_catch(parts)
        self.assertIn("Policy error", notices)
        self.assertEqual((state.app.notice, state.app.notice_level, runner.seen), ("Policy error", "warning", []))


class LoadPickPolicyTests(unittest.TestCase):
    def test_disabled_failed_and_loaded(self):
        self.assertIsNone(load_pick_policy("", "auto", factory=mock.Mock()))
        failing = mock.Mock()
        failing.return_value.load.side_effect = OSError("no network")
        with self.assertLogs("robot.teleop_drive", "WARNING") as logs:
            self.assertIsNone(load_pick_policy("hub/id", "cpu", factory=failing))
        self.assertIn("stub", logs.output[0])
        loaded = mock.Mock()
        self.assertIs(load_pick_policy("hub/id", "mps", factory=loaded), loaded.return_value)
        loaded.assert_called_once_with("hub/id", device="mps", ensemble_coeff=PICK_TEMPORAL_ENSEMBLE_COEFF)
        load_pick_policy("hub/id", "cpu", None, factory=loaded)
        self.assertIsNone(loaded.call_args.kwargs["ensemble_coeff"])


class EnsembleArgsTests(unittest.TestCase):
    def test_default_value_and_off_switch(self):
        self.assertEqual(parse_args([]).pick_ensemble, PICK_TEMPORAL_ENSEMBLE_COEFF)
        self.assertEqual(parse_args(["--pick-ensemble", "0.05"]).pick_ensemble, 0.05)
        self.assertIsNone(parse_args(["--no-pick-ensemble"]).pick_ensemble)

    def test_bad_values_and_both_flags_are_refused(self):
        for argv in (["--pick-ensemble", "-1"], ["--pick-ensemble", "fast"], ["--pick-ensemble", "inf"],
                     ["--pick-ensemble", "0.01", "--no-pick-ensemble"]):
            with self.subTest(argv=argv), self.assertRaises(SystemExit), \
                    mock.patch("sys.stderr"):  # argparse prints the usage error
                parse_args(argv)


if __name__ == "__main__":
    unittest.main()
