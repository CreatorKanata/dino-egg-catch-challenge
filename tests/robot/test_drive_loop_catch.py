"""tests/robot/test_drive_loop_catch.py: Loop wiring of the Auto Catch arm phases (Phase 3 step 1).

Uses the fake devices in loop_fakes.py with the egg detector returning an egg at the target (so the
alignment ends quickly), the wrist check patched, and the recorded poses in a temporary directory.
Verified without hardware or OpenCV: the arm moving slowly to the catch pose and back to the
release pose, the wrist check running only during its frames, a disabled check proceeding to the
policy stub, an enabled check without an egg returning to the release pose, an arm left off the
release pose moving there first (base at zero) before the alignment, the missing catch pose refusal, Stop in an arm phase, the phase on the signboard, and the catch-pose staff key. A fake
monotonic clock advances one loop period per frame. Skipped when LeRobot is unavailable.
"""

import unittest
from unittest import mock

try:
    from robot import drive_loop
    from robot.drive_loop import DriveDevices, step
except ImportError:  # pragma: no cover - depends on the environment
    DriveDevices = None

from loop_fakes import CATCH_POSE, DRIVING, FORWARD, FRONT_RGB, HOLD, NEAR, ZEROS, FakeAdapter, FakeLeader
from loop_fakes import FakeReader, FakeView, temp_arm_paths
from robot.arm_motions import load_pose
from robot.auto_catch import CatchLimits
from robot.config import ALIGN_TARGET_CX, ALIGN_TARGET_H, ALIGN_TARGET_W_EGG, ARM_KEYS, LOOP_HZ, SAVE_CATCH_COMMAND
from robot.vision.egg_size import EggDetection, WristView

ON_TARGET = EggDetection(cx=ALIGN_TARGET_CX, cy=0.5, w=ALIGN_TARGET_W_EGG, h=ALIGN_TARGET_H, color="green", spots=3, area_px=20000)
GRIPPER = "arm_gripper.pos"
DISABLED = CatchLimits(wrist_check_enabled=False)
ENABLED = CatchLimits(wrist_check_enabled=True)
SEEN = WristView(color="green", partial=False, cx=0.5, cy=0.5)


class WristAdapter(FakeAdapter):
    """FakeAdapter whose observation also carries a wrist frame."""

    def observe(self):
        return {**super().observe(), "wrist": FRONT_RGB}


class CatchLoopTests(unittest.TestCase):
    def setUp(self):
        if DriveDevices is None:
            self.skipTest("LeRobot is not installed")
        for name, value in (("detect_eggs", (ON_TARGET,)), ("detect_basket", None), ("append_row", None),
                            ("egg_in_wrist_view", None)):
            patcher = mock.patch.object(drive_loop, name, return_value=value)
            setattr(self, name, patcher.start())
            self.addCleanup(patcher.stop)
        self.clock = 100.0
        clock = mock.patch.object(drive_loop.time, "monotonic", side_effect=self.tick)
        clock.start()
        self.addCleanup(clock.stop)
        self.leader, self.view = FakeLeader(NEAR), FakeView(keep_running=True)
        self.paths = temp_arm_paths(self)
        self.use(DISABLED)

    def use(self, limits, paths=None):
        self.parts = DriveDevices(reader=FakeReader(FORWARD), adapter=WristAdapter(), camera=None, view=self.view,
                                  use_rerun=False, leader=self.leader, arm_paths=paths or self.paths,
                                  catch_limits=limits)

    def tick(self):
        self.clock += 1 / LOOP_HZ
        return self.clock

    def run_frame(self, state, *commands):
        self.view.commands = commands
        state, keep_running, _ = step(self.parts, state, self.clock)
        self.assertTrue(keep_running)
        return state

    def run_catch(self, limit=900):
        """Hi! after one detection frame, then frames until the action ends; returns the last state
        and the (phase, notice, arm status) of every frame."""
        state = self.run_frame(self.run_frame(DRIVING), "hi")
        self.assertEqual(state.app.action, "auto_catch")
        reads, frames = self.leader.reads, []
        for _ in range(limit):
            state = self.run_frame(state)
            frames.append((self.view.rendered[-1][2].phase, state.app.notice, state.arm_status))
            if state.app.action == "none":
                break
        self.assertEqual(self.leader.reads, reads)  # the leader is never read during the action
        return state, frames

    def assert_near(self, pose, target, keys=ARM_KEYS):
        self.assertTrue(all(abs(pose[key] - target[key]) <= 3.0 for key in keys), (pose, target))

    def test_disabled_check_runs_the_stub_and_returns_to_the_release_pose(self):
        state, frames = self.run_catch()
        phases = [phase for phase, _, _ in frames]
        for phase in ("align", "to_catch", "wrist_check", "to_release"):
            self.assertIn(phase, phases)
        notices = {notice for _, notice, _ in frames}
        self.assertIn("Catch: policy not available yet", notices)
        self.assertEqual((state.app.notice, state.app.notice_level), ("Ready", "info"))
        self.assertTrue(any(all(abs(arm[key] - CATCH_POSE[key]) <= 3.0 for key in ARM_KEYS)
                            for arm in self.parts.adapter.arms))  # reached the catch pose
        final = self.parts.adapter.arms[-1]
        self.assert_near(final, HOLD, [key for key in ARM_KEYS if key != GRIPPER])
        self.assertAlmostEqual(final[GRIPPER], CATCH_POSE[GRIPPER], delta=3.0)  # gripper kept on the way back
        self.assertEqual(self.egg_in_wrist_view.call_count, 0)  # disabled: the check never runs
        self.assertEqual(self.parts.adapter.sent[-1], ZEROS)
        self.assertTrue(all(status == "auto catch" for _, _, status in frames[:-1]))
        state = self.run_frame(state)
        self.assertEqual(state.arm_status, "syncing")  # slow re-sync to the leader
        self.assertAlmostEqual(self.parts.adapter.sent[-1]["x.vel"], 0.1)  # driving again

    def test_enabled_check_without_an_egg_returns_to_the_release_pose(self):
        self.use(ENABLED)
        state, frames = self.run_catch()
        notices = [notice for _, notice, _ in frames]
        self.assertIn("Egg not in wrist view", notices)
        self.assertNotIn("Catch: policy not available yet", notices)
        self.assertNotIn("Ready", notices)  # the warning survives the return to the release pose
        self.assertEqual((state.app.notice, state.app.notice_level), ("Egg not in wrist view", "warning"))
        self.assertEqual(self.egg_in_wrist_view.call_count, ENABLED.wrist_check_frames)  # only during the check
        self.assert_near(self.parts.adapter.arms[-1], HOLD, [key for key in ARM_KEYS if key != GRIPPER])

    def test_enabled_check_with_an_egg_goes_to_the_stub(self):
        self.use(ENABLED)
        self.egg_in_wrist_view.return_value = SEEN
        _, frames = self.run_catch()
        notices = [notice for _, notice, _ in frames]
        self.assertIn("Catch: policy not available yet", notices)
        self.assertNotIn("Egg not in wrist view", notices)

    def test_arm_off_the_release_pose_moves_there_before_aligning(self):
        state = self.run_frame(DRIVING)
        self.parts.adapter.arm_hold = {**HOLD, "arm_wrist_flex.pos": 45.0}  # the leader left the head low
        state = self.run_frame(state, "hi")
        self.assertEqual(state.app.catch.phase, "to_start")
        phases = []
        for _ in range(300):
            state = self.run_frame(state)
            phases.append(state.app.catch.phase)
            if state.app.catch.phase == "align":
                break
            self.assertEqual(self.parts.adapter.sent[-1], ZEROS)  # the base never moves while the arm does
        self.assertEqual(self.view.rendered[-2][2].phase, "to_start")
        self.assert_near(self.parts.adapter.arms[-1], HOLD, [key for key in ARM_KEYS if key != GRIPPER])
        self.assertEqual(phases[-1], "align")

    def test_missing_catch_pose_refuses_and_nothing_moves(self):
        self.use(DISABLED, temp_arm_paths(self, catch=None))
        with self.assertLogs("robot.arm_store", level="WARNING"):
            state = self.run_frame(self.run_frame(DRIVING), "hi")
        self.assertEqual((state.app.action, state.app.notice, state.app.notice_level),
                         ("none", "Catch pose not recorded", "warning"))
        self.assertAlmostEqual(self.parts.adapter.sent[-1]["x.vel"], 0.1)  # still driving by hand

    def test_stop_in_an_arm_phase_zeros_and_holds(self):
        state = self.run_frame(self.run_frame(DRIVING), "hi")
        for _ in range(300):
            state = self.run_frame(state)
            if state.app.catch.phase == "to_catch" and self.parts.adapter.arms[-1] != self.parts.adapter.arms[-2]:
                break
        held = self.parts.adapter.arms[-1]
        state = self.run_frame(self.run_frame(state, "stop"))
        self.assertEqual((state.app.action, state.app.stopped, state.arm_status), ("none", True, "holding"))
        self.assertEqual(self.parts.adapter.sent[-2:], [ZEROS, ZEROS])
        self.assertEqual(self.parts.adapter.arms[-2:], [held, held])

    def test_catch_key_saves_the_commanded_pose(self):
        self.paths.catch.unlink()
        state = self.run_frame(DRIVING)  # the leader engages (NEAR)
        state = self.run_frame(state, SAVE_CATCH_COMMAND)
        self.assertEqual((state.app.notice, load_pose(self.paths.catch)), ("Catch pose saved", NEAR))
        self.assertEqual(load_pose(self.paths.home), HOLD)  # the release pose (home) is untouched


if __name__ == "__main__":
    unittest.main()
