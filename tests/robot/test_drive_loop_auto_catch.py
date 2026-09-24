"""tests/robot/test_drive_loop_auto_catch.py: Loop wiring of the Auto Catch alignment and captures.

Uses the fake devices in loop_fakes.py with the egg detector patched to return chosen
detections and the recorded poses in a temporary directory, so the Hi! precondition alerts, the
alignment driving the base (controller and leader ignored, input loss not stopping it), its done /
lost results, Stop cancelling it, the capture command, and the alignment trace rows are verified
without hardware or OpenCV. The phases after the alignment are in test_drive_loop_catch.py. A fake
monotonic clock advances one loop period per frame, so the rate-limited ramp is predictable.
Skipped when LeRobot is unavailable.
"""

from dataclasses import replace
import unittest
from unittest import mock

import numpy as np

try:
    from robot import drive_loop
    from robot.drive_loop import DriveDevices, step
except ImportError:  # pragma: no cover - depends on the environment
    DriveDevices = None

from loop_fakes import DRIVING, FORWARD, FRONT_BGR, NEAR, ZEROS, FakeAdapter, FakeLeader, FakeReader, FakeView
from loop_fakes import temp_arm_paths
from robot.config import (
    ALIGN_DONE_FRAMES,
    ALIGN_LOST_FRAMES,
    ALIGN_MAX_ACCEL,
    ALIGN_TARGET_CX,
    ALIGN_TARGET_W_EGG,
    LOOP_HZ,
)
from robot.vision.egg_size import EggDetection

FAR_RIGHT = EggDetection(cx=0.8, cy=0.5, w=0.4, h=0.4, color="green", spots=3, area_px=20000)
ON_TARGET = replace(FAR_RIGHT, cx=ALIGN_TARGET_CX, w=ALIGN_TARGET_W_EGG)  # no ellipse: ellipse_w = bbox w
TINY = replace(FAR_RIGHT, h=0.05)


class AutoCatchLoopTests(unittest.TestCase):
    def setUp(self):
        if DriveDevices is None:
            self.skipTest("LeRobot is not installed")
        patcher = mock.patch.object(drive_loop, "detect_eggs", return_value=(FAR_RIGHT,))
        self.detector = patcher.start()
        self.addCleanup(patcher.stop)
        basket = mock.patch.object(drive_loop, "detect_basket", return_value=None)
        basket.start()
        self.addCleanup(basket.stop)
        trace = mock.patch.object(drive_loop, "append_row")  # never write captures/ in tests
        self.trace = trace.start()
        self.addCleanup(trace.stop)
        self.clock = 100.0
        clock = mock.patch.object(drive_loop.time, "monotonic", side_effect=self.tick)
        clock.start()
        self.addCleanup(clock.stop)
        self.leader = FakeLeader(NEAR)
        self.view = FakeView(keep_running=True)
        self.reader = FakeReader(FORWARD)
        self.parts = DriveDevices(reader=self.reader, adapter=FakeAdapter(), camera=None, view=self.view,
                                  use_rerun=False, leader=self.leader, arm_paths=temp_arm_paths(self))

    def tick(self):
        self.clock += 1 / LOOP_HZ
        return self.clock

    def run_frame(self, state, *commands):
        self.view.commands = commands
        state, keep_running, _ = step(self.parts, state, self.clock)
        self.assertTrue(keep_running)
        return state

    def start_aligning(self):
        state = self.run_frame(DRIVING)  # detects the egg for the next frame's Hi! check
        reads = self.leader.reads
        state = self.run_frame(state, "hi")
        self.assertEqual(self.leader.reads, reads)  # leader ignored from the Hi! frame on
        self.assertEqual(self.parts.adapter.sent[-1], ZEROS)  # start frame: dt = 0, no motion yet
        return self.run_frame(state)

    def test_rejected_hi_shows_warning_and_keeps_driving(self):
        for detections, notice in (((), "No egg in view"), ((TINY,), "Egg too far")):
            with self.subTest(notice=notice):
                self.detector.return_value = detections
                state = self.run_frame(self.run_frame(DRIVING), "hi")
                self.assertEqual((state.app.action, state.app.notice, state.app.notice_level),
                                 ("none", notice, "warning"))
                self.assertAlmostEqual(self.parts.adapter.sent[-1]["x.vel"], 0.1)
                self.assertEqual(self.view.rendered[-1][2].notice_level, "warning")

    def test_alignment_drives_the_base_and_holds_the_arm(self):
        state = self.start_aligning()
        sent = self.parts.adapter.sent[-1]
        self.assertEqual(state.app.action, "auto_catch")
        self.assertLess(sent["y.vel"], 0.0)  # egg right of target -> move right
        self.assertGreater(sent["x.vel"], 0.0)  # egg smaller than target -> forward
        self.assertAlmostEqual(sent["x.vel"], ALIGN_MAX_ACCEL / LOOP_HZ)  # ramping up, not the joystick
        self.assertEqual((state.arm_status, self.parts.adapter.arms[-1]), ("auto catch", NEAR))  # last pose held
        status = self.view.rendered[-1][2]
        self.assertEqual((status.action, status.phase, [overlay.kind for overlay in status.overlays]),
                         ("auto_catch", "align", ["target", "egg_ok"]))
        self.assertEqual(status.overlays[1].label, "green egg")

    def test_controller_input_loss_does_not_stop_alignment(self):
        state = self.start_aligning()
        self.reader.stale = True
        state = self.run_frame(state)
        self.assertEqual(state.app.action, "auto_catch")
        self.assertLess(self.parts.adapter.sent[-1]["y.vel"], 0.0)

    def test_done_moves_on_to_the_catch_pose_with_zeros(self):
        state = self.start_aligning()
        self.detector.return_value = (ON_TARGET,)
        reads = self.leader.reads
        for _ in range(ALIGN_DONE_FRAMES + 10):  # smoothing converges first, then N frames in tolerance
            state = self.run_frame(state)
            if state.app.catch.phase != "align":
                break
        self.assertEqual((state.app.action, state.app.catch.phase), ("auto_catch", "to_catch"))
        self.assertEqual(self.parts.adapter.sent[-1], ZEROS)
        self.assertEqual((self.leader.reads, state.arm_status, state.align_trace), (reads, "auto catch", None))

    def test_egg_lost(self):
        state = self.start_aligning()
        for _ in range(5):
            state = self.run_frame(state)  # build up some speed
        self.detector.return_value = ()
        for _ in range(ALIGN_LOST_FRAMES + 1):
            state = self.run_frame(state)
        self.assertEqual((state.app.action, state.app.notice, state.app.notice_level),
                         ("none", "Egg lost", "warning"))
        self.assertEqual(self.parts.adapter.sent[-1], ZEROS)
        misses = [sent["y.vel"] for sent in self.parts.adapter.sent[-ALIGN_LOST_FRAMES - 1:-1]]
        self.assertTrue(all(y_vel < 0 for y_vel in misses))  # kept steering on the last known egg

    def test_short_flicker_keeps_the_command_sign(self):
        state = self.start_aligning()
        for detections in ((), (), (FAR_RIGHT,), ()):
            self.detector.return_value = detections
            state = self.run_frame(state)
            self.assertEqual(state.app.action, "auto_catch")
            self.assertLess(self.parts.adapter.sent[-1]["y.vel"], 0.0)

    def test_trace_rows_go_to_one_file_per_alignment(self):
        state = self.run_frame(self.start_aligning(), "stop")
        paths = {call.args[0] for call in self.trace.call_args_list}
        self.assertEqual(len(paths), 1)
        self.assertTrue(str(paths.pop()).endswith("-align.csv"))
        self.assertEqual(self.trace.call_count, 2)  # start frame + one running frame; Stop ends it
        self.assertIsNone(state.align_trace)

    def test_trace_write_failure_does_not_stop_alignment(self):
        self.trace.side_effect = OSError("read-only")
        with self.assertLogs("robot.drive_loop", level="ERROR") as logs:
            state = self.start_aligning()
            state = self.run_frame(state)
        self.assertEqual((state.app.action, state.align_trace), ("auto_catch", ""))
        self.assertEqual(len(logs.output), 1)  # logged once, then tracing is off for this run

    def test_stop_cancels_alignment(self):
        state = self.run_frame(self.start_aligning(), "stop")
        self.assertEqual((state.app.action, state.app.stopped), ("none", True))
        self.assertEqual(self.parts.adapter.sent[-1], ZEROS)
        state = self.run_frame(state)
        self.assertEqual(self.parts.adapter.sent[-1], ZEROS)


class CaptureLoopTests(unittest.TestCase):
    def setUp(self):
        if DriveDevices is None:
            self.skipTest("LeRobot is not installed")
        patcher = mock.patch.object(drive_loop, "detect_eggs", return_value=(FAR_RIGHT,))
        patcher.start()
        self.addCleanup(patcher.stop)
        basket = mock.patch.object(drive_loop, "detect_basket", return_value=None)
        basket.start()
        self.addCleanup(basket.stop)
        self.view = FakeView(keep_running=True, commands=("capture",))
        self.parts = DriveDevices(reader=FakeReader(FORWARD), adapter=FakeAdapter(), camera=None, view=self.view,
                                  use_rerun=False)

    def test_capture_saves_bgr_frames_and_shows_notice(self):
        with mock.patch.object(drive_loop, "save_capture", return_value="captures/x.json") as save:
            state, _, _ = step(self.parts, DRIVING, None)
        frames, detections, mode = save.call_args.args
        np.testing.assert_array_equal(frames["front"], FRONT_BGR)
        self.assertEqual((detections, mode), ((FAR_RIGHT,), "manual"))
        self.assertEqual((state.app.notice, state.app.notice_level), ("Captured", "info"))
        self.assertAlmostEqual(self.parts.adapter.sent[0]["x.vel"], 0.1)  # driving is unaffected

    def test_failed_capture_is_shown_not_raised(self):
        with mock.patch.object(drive_loop, "save_capture", side_effect=OSError("disk full")), \
                self.assertLogs("robot.drive_loop", level="ERROR"):
            state, keep_running, _ = step(self.parts, DRIVING, None)
        self.assertTrue(keep_running)
        self.assertEqual((state.app.notice, state.app.notice_level), ("Capture failed", "warning"))


if __name__ == "__main__":
    unittest.main()
