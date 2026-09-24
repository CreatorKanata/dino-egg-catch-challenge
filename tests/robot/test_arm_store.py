"""tests/robot/test_arm_store.py: Checks of the staff keys that record the Auto Release data.

The save-home key writes the commanded pose, the record key starts and stops a recording that is
written as a valid motion file with one backup, recording is refused outside Manual Mode or when
the arm is not following and cancelled when following stops, write failures are shown and never
raised, and the Thx loader turns missing or invalid files into None. Temporary paths only.
"""

import json
from pathlib import Path
import shutil
import tempfile
import unittest

from robot.arm_motions import load_motion, load_pose
from robot.arm_store import ArmDataPaths, RecordingState, handle_staff_keys, load_release_request, recording_seconds
from robot.arm_store import wants_release
from robot.config import ARM_KEYS, LOOP_HZ, SAVE_HOME_COMMAND, TOGGLE_RECORD_COMMAND
from robot.mode_manager import AppState

POSE = {key: float(index) for index, key in enumerate(ARM_KEYS)}


class StaffKeyTests(unittest.TestCase):
    def setUp(self):
        self.folder = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.folder, ignore_errors=True)
        self.paths = ArmDataPaths(home=self.folder / "arm" / "home_pose.json",
                                  motion=self.folder / "arm" / "release_motion.json")

    def keys(self, app, recording, commands, now, status="following", pose=POSE):
        return handle_staff_keys(app, recording, commands, pose, status, now, self.paths)

    def test_save_home_writes_the_commanded_pose(self):
        app, _ = self.keys(AppState(), RecordingState(), (SAVE_HOME_COMMAND,), 1.0, status="no leader")
        self.assertEqual((app.notice, app.notice_level), ("Home pose saved", "info"))
        self.assertEqual(load_pose(self.paths.home), POSE)

    def test_save_home_ignored_while_an_action_runs(self):
        busy = AppState(action="auto_release")
        self.assertEqual(self.keys(busy, RecordingState(), (SAVE_HOME_COMMAND,), 1.0)[0], busy)
        self.assertFalse(self.paths.home.exists())

    def test_record_toggle_writes_a_valid_motion(self):
        app, recording = self.keys(AppState(), RecordingState(), (TOGGLE_RECORD_COMMAND,), 10.0)
        self.assertEqual((app.notice, recording.active, len(recording.frames)), ("Recording release...", True, 1))
        for frame in range(1, 30):
            pose = {**POSE, "arm_gripper.pos": float(frame)}
            app, recording = self.keys(app, recording, (), 10.0 + frame / LOOP_HZ, pose=pose)
        self.assertAlmostEqual(recording_seconds(recording, 11.0), 1.0)
        app, recording = self.keys(app, recording, (TOGGLE_RECORD_COMMAND,), 11.0)
        self.assertEqual((recording, app.notice), (RecordingState(), "Release motion saved (30 frames, 1.0 s)"))
        motion = load_motion(self.paths.motion)
        self.assertEqual((len(motion.frames), motion.rate_hz, motion.frames[-1].pose["arm_gripper.pos"]), (30, 30.0, 29.0))
        self.assertIn("note", json.loads(self.paths.motion.read_text()))
        self.assertIsNone(recording_seconds(recording, 12.0))

    def test_recording_stops_and_saves_at_the_maximum_length(self):
        now = 50.0
        app, recording = handle_staff_keys(AppState(), RecordingState(), (TOGGLE_RECORD_COMMAND,), POSE, "following",
                                           now, self.paths, max_s=1.0)
        for _ in range(40):  # fake clock: one loop period per frame, past the 1 s cap
            now += 1 / LOOP_HZ
            app, recording = handle_staff_keys(app, recording, (), POSE, "following", now, self.paths, max_s=1.0)
            if not recording.active:
                break
        self.assertEqual((recording, app.notice), (RecordingState(), "Release motion saved (max length)"))
        motion = load_motion(self.paths.motion)
        self.assertGreaterEqual(motion.duration_s, 1.0)
        self.assertLess(motion.duration_s, 1.0 + 1.5 / LOOP_HZ)

    def test_refusals_and_cancel(self):
        refused = ((AppState(mode="fsc"), "following", "Cannot record: not in Manual Mode"),
                   (AppState(), "syncing", "Cannot record: arm not following"),
                   (AppState(), "no leader", "Cannot record: arm not following"),
                   (AppState(stopped=True), "holding", "Cannot record: arm not following"))
        for app, status, notice in refused:
            with self.subTest(notice=notice, status=status):
                after, recording = self.keys(app, RecordingState(), (TOGGLE_RECORD_COMMAND,), 1.0, status)
                self.assertEqual((after.notice, after.notice_level, recording), (notice, "warning", RecordingState()))
        _, recording = self.keys(AppState(), RecordingState(), (TOGGLE_RECORD_COMMAND,), 1.0)
        with self.assertLogs("robot.arm_store", level="WARNING"):
            app, recording = self.keys(AppState(), recording, (), 1.1, status="leader fault")
        self.assertEqual((app.notice, recording), ("Recording cancelled", RecordingState()))
        self.assertFalse(self.paths.motion.exists())
        _, recording = self.keys(AppState(), RecordingState(), (TOGGLE_RECORD_COMMAND,), 2.0)
        stopped = AppState(stopped=True, notice="STOP")
        with self.assertLogs("robot.arm_store", level="WARNING"):
            app, recording = self.keys(stopped, recording, (), 2.1, status="holding")
        self.assertEqual((app, recording), (stopped, RecordingState()))  # the STOP notice stays

    def test_too_short_recording_writes_nothing(self):
        _, recording = self.keys(AppState(), RecordingState(), (TOGGLE_RECORD_COMMAND,), 1.0)
        with self.assertLogs("robot.arm_store", level="WARNING"):
            app, recording = self.keys(AppState(), recording, (TOGGLE_RECORD_COMMAND,), 1.0)
        self.assertEqual((app.notice, app.notice_level), ("Recording too short", "warning"))
        self.assertFalse(self.paths.motion.exists())

    def test_write_failures_are_shown_not_raised(self):
        blocker = self.folder / "file"
        blocker.write_text("not a directory")
        self.paths = ArmDataPaths(home=blocker / "home.json", motion=blocker / "motion.json")
        with self.assertLogs("robot.arm_store", level="ERROR"):
            app, _ = self.keys(AppState(), RecordingState(), (SAVE_HOME_COMMAND,), 1.0)
        self.assertEqual((app.notice, app.notice_level), ("Home pose not saved", "warning"))
        _, recording = self.keys(AppState(), RecordingState(), (TOGGLE_RECORD_COMMAND,), 1.0)
        _, recording = self.keys(AppState(), recording, (), 1.5)
        with self.assertLogs("robot.arm_store", level="ERROR"):
            app, _ = self.keys(AppState(), recording, (TOGGLE_RECORD_COMMAND,), 2.0)
        self.assertEqual(app.notice, "Release motion not saved")


class LoadTests(unittest.TestCase):
    def setUp(self):
        self.folder = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.folder, ignore_errors=True)
        self.paths = ArmDataPaths(home=self.folder / "home.json", motion=self.folder / "motion.json")

    def test_missing_and_invalid_files_become_none(self):
        with self.assertLogs("robot.arm_store", level="WARNING"):
            request = load_release_request(self.paths, "ok")
        self.assertEqual((request.size, request.home, request.frames), ("ok", None, None))
        self.paths.home.write_text("{")
        with self.assertLogs("robot.arm_store", level="ERROR"):
            self.assertIsNone(load_release_request(self.paths, "ok").home)

    def test_loaded_request_has_playback_frames(self):
        handle_staff_keys(AppState(), RecordingState(), (SAVE_HOME_COMMAND,), POSE, "following", 1.0, self.paths)
        _, recording = handle_staff_keys(AppState(), RecordingState(), (TOGGLE_RECORD_COMMAND,), POSE, "following",
                                         1.0, self.paths)
        _, recording = handle_staff_keys(AppState(), recording, (), POSE, "following", 1.5, self.paths)
        handle_staff_keys(AppState(), recording, (TOGGLE_RECORD_COMMAND,), POSE, "following", 2.0, self.paths)
        request = load_release_request(self.paths, "too_small")
        self.assertEqual((request.size, request.home), ("too_small", POSE))
        self.assertGreater(len(request.frames), 2)  # 1 s recorded, played at RELEASE_PLAYBACK_SPEED

    def test_wants_release_only_for_a_thx_that_can_start(self):
        self.assertTrue(wants_release(AppState(), ("thx",)))
        for app, commands in ((AppState(), ("hi",)), (AppState(stopped=True), ("thx",)),
                              (AppState(action="auto_catch"), ("thx",)), (AppState(mode="fsc"), ("thx",))):
            with self.subTest(app=app, commands=commands):
                self.assertFalse(wants_release(app, commands))


if __name__ == "__main__":
    unittest.main()
