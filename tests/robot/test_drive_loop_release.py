"""tests/robot/test_drive_loop_release.py: Loop wiring of Auto Release and the staff recording keys.

Uses the fake devices in loop_fakes.py with the egg and basket detectors patched and the home pose
and release motion in a temporary directory, so `Thx` starting the action (controller and leader
ignored, arm status "auto release"), the full run to "Released!", the refusals, Stop during
playback (zeros, arm held), the overlays and progress on the signboard, and the save-home and
record keys writing their files are verified without hardware or OpenCV. A fake monotonic clock
advances one loop period per frame. Skipped when LeRobot is unavailable.
"""

from pathlib import Path
import shutil
import tempfile
import unittest
from unittest import mock

try:
    from robot import drive_loop
    from robot.drive_loop import DriveDevices, step
except ImportError:  # pragma: no cover - depends on the environment
    DriveDevices = None

from loop_fakes import DRIVING, FORWARD, HOLD, NEAR, ZEROS, FakeAdapter, FakeLeader, FakeReader, FakeView
from robot.arm_motions import ArmFileError, MotionFrame, home_record, load_motion, load_pose, motion_record
from robot.arm_motions import write_record
from robot.arm_store import ArmDataPaths
from robot.config import ARM_KEYS, LOOP_HZ, SAVE_HOME_COMMAND, TOGGLE_RECORD_COMMAND
from robot.vision.basket_size import BasketDetection

GRIPPER = "arm_gripper.pos"
BASKET = BasketDetection(cx=0.53, cy=0.41, w=0.80, h=0.70, area_fraction=0.3, fill=0.6)
HOME = {**HOLD, "arm_elbow_flex.pos": -10.0}
OPEN = {**HOLD, GRIPPER: 30.0}


class ReleaseLoopTests(unittest.TestCase):
    def setUp(self):
        if DriveDevices is None:
            self.skipTest("LeRobot is not installed")
        for name, value in (("detect_eggs", ()), ("detect_basket", BASKET), ("append_row", None)):
            patcher = mock.patch.object(drive_loop, name, return_value=value)
            setattr(self, name, patcher.start())
            self.addCleanup(patcher.stop)
        self.clock = 100.0
        clock = mock.patch.object(drive_loop.time, "monotonic", side_effect=self.tick)
        clock.start()
        self.addCleanup(clock.stop)
        self.folder = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.folder, ignore_errors=True)
        self.paths = ArmDataPaths(home=self.folder / "home_pose.json", motion=self.folder / "release_motion.json")
        write_record(self.paths.home, home_record(HOME, "t", "test"))
        write_record(self.paths.motion, motion_record((MotionFrame(0.0, HOLD), MotionFrame(0.2, OPEN)), 30, "t", "x"))
        self.leader, self.view = FakeLeader(NEAR), FakeView(keep_running=True)
        self.parts = DriveDevices(reader=FakeReader(FORWARD), adapter=FakeAdapter(), camera=None, view=self.view,
                                  use_rerun=False, leader=self.leader, arm_paths=self.paths)

    def tick(self):
        self.clock += 1 / LOOP_HZ
        return self.clock

    def run_frame(self, state, *commands):
        self.view.commands = commands
        state, keep_running, _ = step(self.parts, state, self.clock)
        self.assertTrue(keep_running)
        return state

    def start_release(self):
        state = self.run_frame(DRIVING)  # detects the basket; the leader engages (NEAR)
        self.assertEqual(state.arm_status, "following")
        reads = self.leader.reads
        state = self.run_frame(state, "thx")
        self.assertEqual((state.app.action, state.arm_status, self.leader.reads), ("auto_release", "auto release", reads))
        return state

    def test_thx_runs_the_release_to_completion(self):
        state = self.start_release()
        phases, progress = set(), []
        for _ in range(600):
            state = self.run_frame(state)
            phases.add(state.app.release.phase)
            progress.append(self.view.rendered[-1][2].progress)
            if state.app.action == "none":
                break
            self.assertNotAlmostEqual(self.parts.adapter.sent[-1]["x.vel"], 0.1)  # the joystick is ignored
        self.assertEqual(phases, {"align", "home", "play", "return_home", "idle"})
        self.assertEqual((state.app.notice, state.arm_status), ("Released!", "holding"))
        self.assertEqual(self.parts.adapter.sent[-1], ZEROS)
        self.assertTrue(all(abs(self.parts.adapter.arms[-1][key] - HOME[key]) <= 3.0 for key in ARM_KEYS))
        self.assertIn(1.0, progress)
        self.assertEqual(self.leader.reads, 1)  # only before Thx: never read during the action
        state = self.run_frame(state)
        self.assertAlmostEqual(self.parts.adapter.sent[-1]["x.vel"], 0.1)  # driving again
        self.assertEqual(state.arm_status, "syncing")  # slow re-sync to the leader

    def test_signboard_shows_basket_and_release_target(self):
        state = self.start_release()
        status = self.view.rendered[-1][2]
        self.assertEqual([overlay.kind for overlay in status.overlays], ["target", "release_target", "basket"])
        self.assertEqual((status.action, status.arm_status, status.notice), ("auto_release", "auto release",
                                                                             "Aligning to basket..."))
        self.run_frame(self.run_frame(state, "stop"))
        self.assertEqual([overlay.kind for overlay in self.view.rendered[-1][2].overlays], ["target", "basket"])

    def test_refusals_keep_driving(self):
        self.detect_basket.return_value = None
        state = self.run_frame(self.run_frame(DRIVING), "thx")
        self.assertEqual((state.app.action, state.app.notice), ("none", "Basket not in view"))
        self.assertAlmostEqual(self.parts.adapter.sent[-1]["x.vel"], 0.1)
        self.detect_basket.return_value = BASKET
        state = self.run_frame(state)  # the Thx check uses the previous frame's basket
        self.paths.motion.unlink()
        with self.assertLogs("robot.arm_store", level="WARNING"):
            state = self.run_frame(state, "thx")
        self.assertEqual((state.app.action, state.app.notice), ("none", "Release motion not recorded"))

    def test_stop_during_playback_zeros_and_holds(self):
        state = self.start_release()
        for _ in range(400):
            state = self.run_frame(state)
            if state.app.release.phase == "play" and state.app.release.index > 2:
                break
        held = self.parts.adapter.arms[-1]
        self.assertNotEqual(held, HOME)
        state = self.run_frame(state, "stop")
        state = self.run_frame(state)
        self.assertEqual((state.app.action, state.app.stopped, state.arm_status), ("none", True, "holding"))
        self.assertEqual(self.parts.adapter.sent[-2:], [ZEROS, ZEROS])
        self.assertEqual(self.parts.adapter.arms[-2:], [held, held])

    def test_save_home_and_record_keys_write_files(self):
        self.paths.home.unlink()
        self.paths.motion.unlink()
        state = self.run_frame(DRIVING)
        state = self.run_frame(state, SAVE_HOME_COMMAND, TOGGLE_RECORD_COMMAND)
        self.assertEqual(load_pose(self.paths.home), NEAR)
        self.assertTrue(state.recording.active)
        self.leader.pose = {**NEAR, GRIPPER: 20.0}
        for _ in range(10):
            state = self.run_frame(state)
        status = self.view.rendered[-1][2]
        self.assertAlmostEqual(status.recording_s, 10 / LOOP_HZ)
        state = self.run_frame(state, TOGGLE_RECORD_COMMAND)
        motion = load_motion(self.paths.motion)
        self.assertEqual(len(motion.frames), 11)
        self.assertEqual(set(motion.frames[-1].pose), set(ARM_KEYS))
        self.assertTrue(state.app.notice.startswith("Release motion saved (11 frames"))

    def test_record_refused_without_a_following_arm(self):
        self.parts = DriveDevices(reader=FakeReader(FORWARD), adapter=FakeAdapter(), camera=None, view=self.view,
                                  use_rerun=False, leader=None, arm_paths=self.paths)
        state = self.run_frame(self.run_frame(DRIVING), TOGGLE_RECORD_COMMAND)
        self.assertEqual((state.app.notice, state.recording.active), ("Cannot record: arm not following", False))
        with self.assertRaises(ArmFileError):
            motion_record((), 30, "t", "empty")  # nothing recorded means nothing valid to write


if __name__ == "__main__":
    unittest.main()
