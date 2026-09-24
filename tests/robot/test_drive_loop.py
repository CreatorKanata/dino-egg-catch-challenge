"""tests/robot/test_drive_loop.py: Hardware-free checks of one Manual Mode loop iteration.

Fake devices stand in for the controller, robot, leader arm, camera, and signboard
(DisplaySink), so the wiring (action sent before viewing, KachiButton commands folded in,
Stop and mode switches zeroing the base, leader poses reaching send_action once engaged, the
Pi frames converted to BGR, and ESC/close or a dead signboard ending the loop) is verified
without hardware. The egg detector is patched out, so OpenCV is never loaded here; Auto Catch
wiring is in test_drive_loop_auto_catch.py. Skipped when LeRobot is unavailable.
"""

from dataclasses import replace
import unittest
from unittest import mock

import numpy as np

try:
    from robot import drive_loop
    from robot.drive_loop import DriveDevices, camera_frames, step
except ImportError:  # pragma: no cover - depends on the environment
    DriveDevices = None

from loop_fakes import (
    DRIVING,
    FORWARD,
    FRONT_BGR,
    HOLD,
    NEAR,
    ZEROS,
    FakeAdapter,
    FakeCamera,
    FakeLeader,
    FakeReader,
    FakeView,
)
from robot.arm_follow import ArmFollowState
from robot.config import ARM_KEYS, ENCODER_DEGREES_PER_STEP, LOOP_HZ, SPEED_LEVELS, TOP_CAMERA_KEY
from robot.drive_state import DriveState
from robot.manual_mode import LoopState


def devices(reader, view, camera=None, leader=None):
    return DriveDevices(reader=reader, adapter=FakeAdapter(), camera=camera, view=view, use_rerun=False,
                        leader=leader)


def patch_detector(test):
    """Replace the egg detector (OpenCV) with a stub that finds nothing."""
    patcher = mock.patch.object(drive_loop, "detect_eggs", return_value=())
    detector = patcher.start()
    test.addCleanup(patcher.stop)
    return detector


class DriveLoopStepTests(unittest.TestCase):
    def setUp(self):
        if DriveDevices is None:
            self.skipTest("LeRobot is not installed")
        self.detector = patch_detector(self)

    def test_forward_command_and_signboard_frames(self):
        view = FakeView(keep_running=True)
        parts = devices(FakeReader(FORWARD), view, FakeCamera())
        state, keep_running, _ = step(parts, LoopState(), None)
        self.assertTrue(keep_running)
        self.assertFalse(state.drive.input_lost)
        self.assertAlmostEqual(parts.adapter.sent[0]["x.vel"], 0.1)
        frames, _, status = view.rendered[0]
        self.assertEqual(list(frames), [TOP_CAMERA_KEY, "front", "wrist"])
        self.assertEqual((frames[TOP_CAMERA_KEY], frames["wrist"]), ("top-frame", None))  # top untouched
        np.testing.assert_array_equal(frames["front"], FRONT_BGR)  # Pi RGB converted to BGR
        np.testing.assert_array_equal(self.detector.call_args.args[0], FRONT_BGR)
        self.assertEqual((status.mode, status.arm_status), ("manual", "no leader"))
        self.assertEqual([overlay.kind for overlay in status.overlays], ["target"])

    def test_detector_skipped_in_fsc(self):
        view = FakeView(keep_running=True, commands=("mode_toggle",))
        state, _, _ = step(devices(FakeReader(FORWARD), view), DRIVING, None)
        self.assertEqual(state.app.mode, "fsc")
        self.detector.assert_not_called()
        self.assertEqual(view.rendered[0][2].overlays, ())

    def test_signboard_close_ends_loop_after_sending(self):
        parts = devices(FakeReader(FORWARD, stale=True), FakeView(keep_running=False))
        state, keep_running, _ = step(parts, LoopState(), None)
        self.assertFalse(keep_running)
        self.assertTrue(state.drive.input_lost)
        self.assertEqual(parts.adapter.sent, [ZEROS])

    def test_without_signboard_keeps_running(self):
        parts = devices(FakeReader(FORWARD), view=None)
        self.assertTrue(step(parts, LoopState(), None)[1])

    def test_step_returns_time_and_clamps_dt(self):
        rotating = LoopState(drive=DriveState(input_lost=False, pending_rotation_deg=ENCODER_DEGREES_PER_STEP))
        parts = devices(FakeReader(FORWARD), view=None)
        with mock.patch.object(drive_loop.time, "monotonic", return_value=10.5):
            first, _, first_time = step(parts, rotating, None)
            late, _, late_time = step(parts, rotating, 10.0)  # 0.5 s gap, clamped to 2/30 s
        self.assertEqual((first_time, late_time), (10.5, 10.5))
        theta = SPEED_LEVELS[0].theta
        self.assertEqual(first.drive.pending_rotation_deg, ENCODER_DEGREES_PER_STEP)  # first iteration: dt = 0
        self.assertAlmostEqual(late.drive.pending_rotation_deg, ENCODER_DEGREES_PER_STEP - theta * 2 / LOOP_HZ)
        self.assertEqual(parts.adapter.sent[-1]["theta.vel"], theta)

    def test_encoder_click_rotates_right(self):
        parts = devices(FakeReader(FORWARD, delta=1), view=None)
        state, _, _ = step(parts, LoopState(), None)
        self.assertEqual(state.drive.pending_rotation_deg, -ENCODER_DEGREES_PER_STEP)
        self.assertEqual(parts.adapter.sent[0], {"x.vel": 0.1, "y.vel": 0.0, "theta.vel": -SPEED_LEVELS[0].theta})

    def test_camera_frames_without_top_camera(self):
        self.assertEqual(camera_frames({"front": 1}, None), {TOP_CAMERA_KEY: None, "front": 1, "wrist": None})


class ManualModeStepTests(unittest.TestCase):
    def setUp(self):
        if DriveDevices is None:
            self.skipTest("LeRobot is not installed")
        patch_detector(self)

    def test_stop_zeros_base_clears_rotation_disengages_and_holds_arm(self):
        start = replace(DRIVING, drive=replace(DRIVING.drive, pending_rotation_deg=45.0),
                        follow=ArmFollowState(engaged=True), arm_status="following")
        leader = FakeLeader({key: 50.0 for key in ARM_KEYS})
        view = FakeView(keep_running=True, commands=("stop",))
        parts = devices(FakeReader(FORWARD), view, leader=leader)
        state, keep_running, _ = step(parts, start, None)
        self.assertTrue(keep_running)  # Stop does not close the app
        self.assertEqual(parts.adapter.sent, [ZEROS])
        self.assertEqual(state.drive.pending_rotation_deg, 0.0)
        self.assertFalse(state.follow.engaged)
        self.assertEqual(parts.adapter.arms, [HOLD])
        self.assertEqual(leader.reads, 0)
        self.assertEqual((state.app.stopped, state.arm_status), (True, "holding"))
        self.assertEqual(view.rendered[0][2].notice, "STOP")

    def test_base_stays_zero_while_stopped_until_go_go(self):
        leader = FakeLeader(NEAR)
        view = FakeView(keep_running=True, commands=("stop",))
        parts = devices(FakeReader(FORWARD, delta=1), view, leader=leader)
        state, _, _ = step(parts, DRIVING, None)
        for command in ((), ("hi",), ("thx",), ()):  # several stopped frames, joystick and encoder active
            view.commands = command
            state, keep_running, _ = step(parts, state, 0.0)
            self.assertTrue(keep_running and state.app.stopped)
            self.assertEqual(state.drive.pending_rotation_deg, 0.0)
        self.assertEqual(parts.adapter.sent, [ZEROS] * 5)
        self.assertEqual(parts.adapter.arms, [HOLD] * 5)
        self.assertEqual(leader.reads, 0)
        status = view.rendered[-1][2]
        self.assertTrue(status.stopped)
        view.commands = ("mode_toggle",)  # resume: one more zero frame, arm disengaged
        state, _, _ = step(parts, state, 0.0)
        self.assertEqual((state.app.mode, state.app.stopped, parts.adapter.sent[-1]), ("manual", False, ZEROS))
        state, _, _ = step(parts, state, 0.0)
        self.assertAlmostEqual(parts.adapter.sent[-1]["x.vel"], 0.1)
        self.assertEqual(state.arm_status, "following")

    def test_mode_toggle_zeros_base_in_fsc_and_holds_arm(self):
        leader = FakeLeader(NEAR)
        parts = devices(FakeReader(FORWARD, delta=1), FakeView(True, commands=("mode_toggle",)), leader=leader)
        state, _, _ = step(parts, DRIVING, None)
        self.assertEqual(state.app.mode, "fsc")
        state, _, _ = step(parts, state, 0.0)  # still fsc: controller input is ignored
        self.assertEqual(parts.adapter.sent, [ZEROS, ZEROS])
        self.assertEqual(state.drive.pending_rotation_deg, 0.0)
        self.assertEqual(parts.adapter.arms, [HOLD, HOLD])
        self.assertEqual((leader.reads, state.arm_status), (0, "holding"))

    def test_leader_pose_reaches_send_action_once_engaged(self):
        leader = FakeLeader(NEAR)
        parts = devices(FakeReader(FORWARD), FakeView(True), leader=leader)
        state, _, _ = step(parts, DRIVING, None)
        self.assertTrue(state.follow.engaged)
        self.assertEqual(parts.adapter.arms[-1], NEAR)
        leader.pose = {key: 60.0 for key in ARM_KEYS}  # engaged: follows without rate limit
        state, _, _ = step(parts, state, 0.0)
        self.assertEqual((parts.adapter.arms[-1], state.arm_status), (leader.pose, "following"))

    def test_far_leader_is_approached_slowly(self):
        parts = devices(FakeReader(FORWARD), FakeView(True), leader=FakeLeader({key: 90.0 for key in ARM_KEYS}))
        state, _, _ = step(parts, DRIVING, None)  # dt = 0 on the first frame: no motion yet
        self.assertEqual((parts.adapter.arms[-1], state.arm_status), (HOLD, "syncing"))

    def test_leader_fault_holds_arm_and_keeps_driving(self):
        parts = devices(FakeReader(FORWARD), FakeView(True), leader=FakeLeader(None))
        state, _, _ = step(parts, replace(DRIVING, follow=ArmFollowState(engaged=True)), None)
        self.assertEqual(parts.adapter.arms, [HOLD])
        self.assertAlmostEqual(parts.adapter.sent[0]["x.vel"], 0.1)
        self.assertEqual((state.arm_status, state.follow.engaged), ("leader fault", False))


if __name__ == "__main__":
    unittest.main()
