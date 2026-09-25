"""tests/robot/loop_fakes.py: Fake devices shared by the control-loop tests (not a test module).

Stand-ins for the controller reader, LeKiwi adapter, leader arm, overhead camera, and signboard
(DisplaySink) used by test_drive_loop.py, test_drive_loop_auto_catch.py, test_drive_loop_catch.py, and
test_drive_loop_release.py, plus temporary arm data files (temp_arm_paths). The adapter returns a tiny
RGB-ordered front frame, like LeKiwiClient, so the loop's BGR conversion is observable.
Imported as a top-level module because the suite is discovered with `-s tests/robot`.
"""

from dataclasses import replace
from pathlib import Path
import shutil
import tempfile

import numpy as np

from robot.arm_motions import home_record, write_record
from robot.arm_store import ArmDataPaths
from robot.config import ARM_KEYS
from robot.dino_controller_reader import INITIAL_STATE
from robot.drive_state import DriveState
from robot.manual_mode import LoopState

FORWARD = replace(INITIAL_STATE, synchronized=True, up=True, last_update_monotonic=0.0)
ZEROS = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
HOLD = {key: 0.0 for key in ARM_KEYS}
NEAR = {key: 1.0 for key in ARM_KEYS}  # within engagement tolerance of HOLD
DRIVING = LoopState(drive=DriveState(input_lost=False))
FRONT_RGB = np.array([[[255, 0, 0], [0, 0, 255]]], dtype=np.uint8)  # 1x2: red, blue (RGB order)
FRONT_BGR = FRONT_RGB[..., ::-1]
TOP_SMALL = np.zeros((9, 16, 3), dtype=np.uint8)  # 16:9, already below TOP_DISPLAY_WIDTH
CATCH_POSE = {**HOLD, "arm_elbow_flex.pos": 6.0, "arm_gripper.pos": 30.0}  # head down, mouth open


def temp_arm_paths(test, home=HOLD, catch=CATCH_POSE):
    """ArmDataPaths in a temporary directory (removed after the test) with the given poses written."""
    folder = Path(tempfile.mkdtemp())
    test.addCleanup(shutil.rmtree, folder, ignore_errors=True)
    paths = ArmDataPaths(home=folder / "home_pose.json", motion=folder / "release_motion.json",
                         catch=folder / "catch_pose.json")
    for path, pose in ((paths.home, home), (paths.catch, catch)):
        if pose is not None:
            write_record(path, home_record(pose, "t", "test"))
    return paths


class FakeReader:
    def __init__(self, state, stale=False, delta=0):
        self.state, self.stale, self.delta = state, stale, delta

    def poll(self, now):
        return self.state, self.delta

    def is_stale(self, now):
        return self.stale


class FakeAdapter:
    """Mimics LeKiwiAdapter.send_action(base, arm_pose, arm_torque): records and holds the last arm
    pose and records the torque flags; capture_hold re-reads the hold from an observation. Set
    `pose` to add arm positions to the observations."""

    def __init__(self):
        self.sent, self.arms, self.torques, self.arm_hold = [], [], [], dict(HOLD)
        self.pose, self.captured = None, []

    def send_action(self, base, arm_pose=None, arm_torque=True):
        self.sent.append(dict(base))
        self.arm_hold = dict(self.arm_hold if arm_pose is None else arm_pose)
        self.arms.append(self.arm_hold)
        self.torques.append(arm_torque)

    def capture_hold(self, observation):
        self.captured.append(observation)
        if all(key in observation for key in ARM_KEYS):
            self.arm_hold = {key: float(observation[key]) for key in ARM_KEYS}

    def observe(self):
        return {"front": FRONT_RGB, "wrist": None, "x.vel": 0.0, **(self.pose or {})}


class FakeLeader:
    def __init__(self, pose):
        self.pose, self.reads = pose, 0

    def read_pose(self):
        self.reads += 1
        return None if self.pose is None else dict(self.pose)


class FakeView:
    def __init__(self, keep_running, commands=()):
        self.keep_running, self.rendered, self.commands = keep_running, [], tuple(commands)

    def render(self, frames, drive, controller, status):
        self.rendered.append((frames, drive, status))

    def pump(self):
        return self.keep_running

    def poll_commands(self):
        commands, self.commands = self.commands, ()
        return commands

    def close(self):
        self.keep_running = False


class FakeCamera:
    """Overhead camera returning a frame narrower than TOP_DISPLAY_WIDTH, so no resize (no cv2)."""

    def __init__(self):
        self.frame = TOP_SMALL

    def read_latest(self):
        return self.frame
