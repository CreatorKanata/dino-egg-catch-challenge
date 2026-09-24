"""tests/robot/test_arm_motions.py: Checks of the home pose and release motion files (Auto Release).

arm_motions.py is stdlib-only, so loading and validation (six keys, finite numbers, strictly
increasing t, at least two frames, version), linear resampling, time scaling, playback frames, and
the atomic writes with one backup are verified with temporary files, never data/arm/.
"""

import json
from pathlib import Path
import shutil
import tempfile
import unittest

from robot.arm_motions import (
    ArmFileError,
    ArmMotion,
    MotionFrame,
    home_record,
    load_motion,
    load_pose,
    motion_record,
    parse_motion,
    playback_frames,
    resample,
    time_scale,
    write_record,
)
from robot.config import ARM_KEYS

ZERO = {key: 0.0 for key in ARM_KEYS}
TEN = {key: 10.0 for key in ARM_KEYS}
MOTION = ArmMotion(rate_hz=30, frames=(MotionFrame(0.0, ZERO), MotionFrame(1.0, TEN)))


def motion_doc(frames=((0.0, ZERO), (1.0, TEN)), **extra):
    return {"version": 1, "recorded_at": "2026-09-24T23:00:00+09:00", "note": "test", "rate_hz": 30,
            "frames": [{"t": t, "pose": pose} for t, pose in frames], **extra}


class FileTests(unittest.TestCase):
    def setUp(self):
        self.folder = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.folder, ignore_errors=True)

    def write(self, name, document):
        path = self.folder / name
        path.write_text(document if isinstance(document, str) else json.dumps(document))
        return path

    def test_home_round_trip_and_note(self):
        path = self.folder / "arm" / "home_pose.json"
        write_record(path, home_record(TEN, "2026-09-24T23:00:00+09:00", "neck folded"))
        self.assertEqual(load_pose(path), TEN)
        self.assertEqual(json.loads(path.read_text())["note"], "neck folded")

    def test_motion_round_trip(self):
        path = self.write("release_motion.json", motion_doc())
        motion = load_motion(path)
        self.assertEqual((motion.rate_hz, len(motion.frames), motion.duration_s), (30.0, 2, 1.0))
        self.assertEqual(motion.frames[1].pose, TEN)

    def test_missing_file_raises_os_error(self):
        with self.assertRaises(FileNotFoundError):
            load_pose(self.folder / "missing.json")

    def test_malformed_home_files_are_rejected(self):
        bad_pose = {"version": 1, "pose": {**TEN, "arm_gripper.pos": "open"}}
        cases = {"not json": "{", "list": [], "no version": {"pose": TEN}, "version 2": {"version": 2, "pose": TEN},
                 "five keys": {"version": 1, "pose": {key: 1.0 for key in ARM_KEYS[:5]}},
                 "extra key": {"version": 1, "pose": {**TEN, "x.vel": 0.0}}, "text value": bad_pose,
                 "bool value": {"version": 1, "pose": {**TEN, "arm_gripper.pos": True}}}
        for name, document in cases.items():
            with self.subTest(name=name), self.assertRaises(ArmFileError):
                load_pose(self.write("home.json", document))

    def test_malformed_motions_are_rejected(self):
        cases = {
            "one frame": motion_doc(frames=((0.0, ZERO),)),
            "equal t": motion_doc(frames=((0.0, ZERO), (0.0, TEN))),
            "decreasing t": motion_doc(frames=((1.0, ZERO), (0.5, TEN))),
            "nan t": motion_doc(frames=((0.0, ZERO), (float("nan"), TEN))),
            "bad pose": motion_doc(frames=((0.0, ZERO), (1.0, {"arm_gripper.pos": 1.0}))),
            "zero rate": motion_doc(rate_hz=0),
            "frames not a list": motion_doc(frames=()) | {"frames": "x"},
            "frame not an object": motion_doc() | {"frames": [1, 2]},
        }
        for name, document in cases.items():
            with self.subTest(name=name), self.assertRaises(ArmFileError):
                parse_motion(document)

    def test_motion_write_keeps_one_backup(self):
        path = self.folder / "release_motion.json"
        first = motion_record(MOTION.frames, 30, "t1", "first")
        second = motion_record((MotionFrame(0.0, TEN), MotionFrame(0.5, ZERO)), 30, "t2", "second")
        write_record(path, first, backup=True)
        self.assertFalse((self.folder / "release_motion.prev.json").exists())
        write_record(path, second, backup=True)
        self.assertEqual(json.loads(path.read_text())["note"], "second")
        self.assertEqual(json.loads((self.folder / "release_motion.prev.json").read_text())["note"], "first")
        self.assertEqual(sorted(p.name for p in self.folder.iterdir()), ["release_motion.json", "release_motion.prev.json"])

    def test_motion_record_validates(self):
        with self.assertRaises(ArmFileError):
            motion_record((MotionFrame(0.0, ZERO),), 30, "t", "too short")


class ResampleTests(unittest.TestCase):
    def test_linear_interpolation_and_exact_end(self):
        poses = resample(MOTION, 0.25)
        self.assertEqual(len(poses), 5)
        self.assertEqual([pose["arm_elbow_flex.pos"] for pose in poses], [0.0, 2.5, 5.0, 7.5, 10.0])

    def test_end_is_appended_when_dt_does_not_divide_the_duration(self):
        poses = resample(MOTION, 0.4)
        self.assertEqual([round(pose["arm_gripper.pos"], 6) for pose in poses], [0.0, 4.0, 8.0, 10.0])

    def test_several_segments(self):
        motion = ArmMotion(30, (MotionFrame(0.0, ZERO), MotionFrame(0.1, TEN), MotionFrame(0.3, ZERO)))
        values = [round(pose["arm_shoulder_pan.pos"], 6) for pose in resample(motion, 0.05)]
        self.assertEqual(values, [0.0, 5.0, 10.0, 7.5, 5.0, 2.5, 0.0])

    def test_time_scale_and_playback_frames(self):
        slow = time_scale(MOTION, 0.5)
        self.assertEqual((slow.duration_s, MOTION.duration_s), (2.0, 1.0))  # input unchanged
        self.assertEqual(len(playback_frames(MOTION, speed=0.5, rate_hz=30)), 61)
        self.assertEqual(len(playback_frames(MOTION, speed=1.0, rate_hz=30)), 31)

    def test_invalid_arguments(self):
        for call in (lambda: resample(MOTION, 0.0), lambda: time_scale(MOTION, 0.0)):
            with self.subTest(call=call), self.assertRaises(ValueError):
                call()


if __name__ == "__main__":
    unittest.main()
