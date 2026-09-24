"""tests/robot/test_drive_loop.py: Hardware-free checks of one Drive Mode loop iteration.

Fake devices stand in for the controller, robot, camera, and signboard (DisplaySink), so the
wiring (action sent before viewing, frames handed to the display, ESC/close or a dead
signboard ending the loop) is verified without hardware. Skipped when LeRobot is unavailable.
"""

from dataclasses import replace
import unittest
from unittest import mock

try:
    from robot import drive_loop
    from robot.drive_loop import DriveDevices, camera_frames, step
except ImportError:  # pragma: no cover - depends on the environment
    DriveDevices = None

from robot.config import ENCODER_DEGREES_PER_STEP, LOOP_HZ, SPEED_LEVELS, TOP_CAMERA_KEY
from robot.dino_controller_reader import INITIAL_STATE
from robot.drive_state import DriveState

FORWARD = replace(INITIAL_STATE, synchronized=True, up=True, last_update_monotonic=0.0)


class FakeReader:
    def __init__(self, state, stale=False, delta=0):
        self.state, self.stale, self.delta = state, stale, delta

    def poll(self, now):
        return self.state, self.delta

    def is_stale(self, now):
        return self.stale


class FakeAdapter:
    def __init__(self):
        self.sent = []

    def send_action(self, base):
        self.sent.append(dict(base))

    def observe(self):
        return {"front": "front-frame", "wrist": None, "x.vel": 0.0}


class FakeView:
    def __init__(self, keep_running):
        self.keep_running, self.rendered = keep_running, []

    def render(self, frames, drive, controller):
        self.rendered.append((frames, drive))

    def pump(self):
        return self.keep_running

    def close(self):
        self.keep_running = False


class FakeCamera:
    def read_latest(self):
        return "top-frame"


def devices(reader, view, camera=None):
    return DriveDevices(reader=reader, adapter=FakeAdapter(), camera=camera, view=view, use_rerun=False)


class DriveLoopStepTests(unittest.TestCase):
    def setUp(self):
        if DriveDevices is None:
            self.skipTest("LeRobot is not installed")

    def test_forward_command_and_signboard_frames(self):
        view = FakeView(keep_running=True)
        parts = devices(FakeReader(FORWARD), view, FakeCamera())
        drive, keep_running, _ = step(parts, DriveState(), None)
        self.assertTrue(keep_running)
        self.assertFalse(drive.input_lost)
        self.assertAlmostEqual(parts.adapter.sent[0]["x.vel"], 0.1)
        frames, _ = view.rendered[0]
        self.assertEqual(frames, {TOP_CAMERA_KEY: "top-frame", "front": "front-frame", "wrist": None})

    def test_signboard_close_ends_loop_after_sending(self):
        parts = devices(FakeReader(FORWARD, stale=True), FakeView(keep_running=False))
        drive, keep_running, _ = step(parts, DriveState(), None)
        self.assertFalse(keep_running)
        self.assertTrue(drive.input_lost)
        self.assertEqual(parts.adapter.sent, [{"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}])

    def test_without_signboard_keeps_running(self):
        parts = devices(FakeReader(FORWARD), view=None)
        self.assertTrue(step(parts, DriveState(), None)[1])

    def test_step_returns_time_and_clamps_dt(self):
        rotating = DriveState(input_lost=False, pending_rotation_deg=ENCODER_DEGREES_PER_STEP)
        parts = devices(FakeReader(FORWARD), view=None)
        with mock.patch.object(drive_loop.time, "monotonic", return_value=10.5):
            first, _, first_time = step(parts, rotating, None)
            late, _, late_time = step(parts, rotating, 10.0)  # 0.5 s gap, clamped to 2/30 s
        self.assertEqual((first_time, late_time), (10.5, 10.5))
        theta = SPEED_LEVELS[0].theta
        self.assertEqual(first.pending_rotation_deg, ENCODER_DEGREES_PER_STEP)  # first iteration: dt = 0
        self.assertAlmostEqual(late.pending_rotation_deg, ENCODER_DEGREES_PER_STEP - theta * 2 / LOOP_HZ)
        self.assertEqual(parts.adapter.sent[-1]["theta.vel"], theta)

    def test_encoder_click_rotates_right(self):
        parts = devices(FakeReader(FORWARD, delta=1), view=None)
        drive, _, _ = step(parts, DriveState(), None)
        self.assertEqual(drive.pending_rotation_deg, -ENCODER_DEGREES_PER_STEP)
        self.assertEqual(parts.adapter.sent[0], {"x.vel": 0.1, "y.vel": 0.0, "theta.vel": -SPEED_LEVELS[0].theta})

    def test_camera_frames_without_top_camera(self):
        self.assertEqual(camera_frames({"front": 1}, None), {TOP_CAMERA_KEY: None, "front": 1, "wrist": None})


if __name__ == "__main__":
    unittest.main()
