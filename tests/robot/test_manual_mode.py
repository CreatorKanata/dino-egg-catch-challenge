"""tests/robot/test_manual_mode.py: Hardware-free checks of the per-frame Manual Mode decisions.

manual_mode.py is stdlib-only, so command folding (including `Hi!` with the egg size and the
recorded poses), the Auto Catch step, the base decision (alignment command while it runs, else driving
only in Manual Mode and never in a stop frame), and the arm decision (torque off, no leader, held,
fault, syncing, following) are verified directly, independent of the loop wiring in test_drive_loop.py.
"""

from dataclasses import replace
import unittest

from robot.align import AlignState
from robot.arm_follow import ArmFollowState
from robot.auto_catch import CatchRequest, start_catch
from robot.config import ALIGN_TARGET_CX, ALIGN_TARGET_W_EGG, ARM_KEYS, NOTICE_SECONDS
from robot.dino_controller_reader import INITIAL_STATE
from robot.drive_state import DriveState
from robot.manual_mode import CommandResult, fold_commands, leader_wanted, plan_arm, plan_base, step_auto_catch
from robot.mode_manager import AppState
from robot.vision.egg_size import EggDetection

FORWARD = replace(INITIAL_STATE, synchronized=True, up=True)
DRIVE = DriveState(input_lost=False, pending_rotation_deg=10.0)
ZEROS = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
HOLD = {key: 0.0 for key in ARM_KEYS}
NEAR = {key: 1.0 for key in ARM_KEYS}
FAR = {key: 90.0 for key in ARM_KEYS}
ALIGNING = AppState(action="auto_catch", catch=start_catch(0.0, HOLD, NEAR, NEAR))
READY = CatchRequest(size="ok", catch=NEAR, home=HOLD, arm=HOLD)
FAR_EGG = EggDetection(cx=0.8, cy=0.5, w=0.2, h=0.3, color="green", spots=3, area_px=5000)


class FoldCommandsTests(unittest.TestCase):
    def test_no_commands_only_expires_notice(self):
        noticed = AppState(notice="STOP", notice_until=1.0)
        result = fold_commands(noticed, (), 2.0)
        self.assertEqual((result.app.notice, result.stop_base, result.disengage_arm), ("", False, False))

    def test_commands_apply_in_order_and_flags_accumulate(self):
        result = fold_commands(AppState(), ("stop", "hi"), 5.0)
        self.assertTrue(result.stop_base and result.disengage_arm)
        self.assertTrue(result.app.stopped)  # "hi" is ignored while stopped
        self.assertTrue(fold_commands(AppState(), ("stop", "mode_toggle"), 5.0).app.stopped)  # Go Go! ignored
        resumed = fold_commands(AppState(), ("stop", "mode_manual"), 5.0)
        self.assertEqual((resumed.app.stopped, resumed.app.mode, resumed.app.notice), (False, "manual", "MANUAL"))
        self.assertEqual(resumed.app.notice_until, 5.0 + NOTICE_SECONDS)
        self.assertEqual(fold_commands(AppState(), ("mode_toggle", "mode_toggle"), 0.0).app.mode, "manual")

    def test_hi_uses_the_egg_size_in_manual_mode(self):
        self.assertEqual(fold_commands(AppState(), ("hi",), 1.0).app.notice, "No egg in view")
        self.assertEqual(fold_commands(AppState(), ("hi",), 1.0, CatchRequest(size="too_small")).app.notice,
                         "Egg too far")
        started = fold_commands(AppState(), ("hi",), 1.0, READY)
        self.assertEqual((started.app.action, started.app.catch.phase), ("auto_catch", "align"))
        self.assertTrue(started.disengage_arm)

    def test_hi_in_fsc_still_toggles_voice_and_capture_is_a_no_op(self):
        self.assertTrue(fold_commands(AppState(mode="fsc"), ("hi",), 1.0, READY).app.voice_listening)
        self.assertEqual(fold_commands(AppState(), ("capture",), 1.0).app, AppState())


class StepAutoCatchTests(unittest.TestCase):
    def test_no_action_passes_through(self):
        result = CommandResult(app=AppState(), stop_base=False, disengage_arm=False)
        self.assertEqual(step_auto_catch(result, FAR_EGG, None, HOLD, 1.0), (result, None, None, None))

    def test_running_alignment_commands_the_base_and_holds_the_arm(self):
        result, base, arm, row = step_auto_catch(CommandResult(ALIGNING, False, False), FAR_EGG, None, HOLD, 1.0)
        self.assertEqual((result.app.action, row.result, row.cx_raw), ("auto_catch", "running", FAR_EGG.cx))
        self.assertLess(base["y.vel"], 0.0)  # egg right of target -> move right
        self.assertGreater(base["x.vel"], 0.0)  # egg smaller than target -> forward
        self.assertEqual(arm, HOLD)
        self.assertFalse(result.stop_base or result.disengage_arm)

    def test_done_alignment_moves_on_and_timeout_ends_the_action(self):
        on_target = replace(FAR_EGG, cx=ALIGN_TARGET_CX, w=ALIGN_TARGET_W_EGG)
        almost = replace(ALIGNING, catch=replace(ALIGNING.catch, align=AlignState("aligning", 0.0, ok_frames=100)))
        result, base, _, row = step_auto_catch(CommandResult(almost, False, False), on_target, None, HOLD, 1.0)
        self.assertEqual((row.result, result.app.action, result.app.catch.phase), ("done", "auto_catch", "to_catch"))
        self.assertEqual(base, ZEROS)
        self.assertFalse(result.stop_base)
        timed_out, base, _, _ = step_auto_catch(CommandResult(ALIGNING, False, False), FAR_EGG, None, HOLD, 1000.0)
        self.assertEqual((timed_out.app.notice, timed_out.app.notice_level, base),
                         ("Could not align", "warning", ZEROS))
        self.assertTrue(timed_out.stop_base and timed_out.disengage_arm)


class PlanBaseTests(unittest.TestCase):
    def test_manual_mode_drives(self):
        drive, base = plan_base(DRIVE, FORWARD, AppState(), stopping=False)
        self.assertEqual(drive, DRIVE)
        self.assertAlmostEqual(base["x.vel"], 0.1)

    def test_stop_frame_latch_or_fsc_sends_zeros_and_clears_rotation(self):
        for app, stopping in ((AppState(), True), (AppState(stopped=True), False), (AppState(mode="fsc"), False)):
            with self.subTest(mode=app.mode, stopping=stopping):
                drive, base = plan_base(DRIVE, FORWARD, app, stopping)
                self.assertEqual((drive.pending_rotation_deg, base), (0.0, ZEROS))

    def test_alignment_command_replaces_the_controller_even_without_input(self):
        auto = {"x.vel": 0.05, "y.vel": -0.04, "theta.vel": 0.0}
        lost = replace(DRIVE, input_lost=True)
        drive, base = plan_base(lost, FORWARD, ALIGNING, stopping=False, auto_base=auto)
        self.assertEqual((drive.pending_rotation_deg, base), (0.0, auto))
        self.assertEqual(plan_base(DRIVE, FORWARD, ALIGNING, stopping=True, auto_base=auto)[1], ZEROS)
        self.assertEqual(plan_base(DRIVE, FORWARD, ALIGNING, stopping=False)[1], ZEROS)  # no command: zeros


class PlanArmTests(unittest.TestCase):
    def test_leader_wanted_only_in_unstopped_manual_mode(self):
        self.assertTrue(leader_wanted(AppState(), True))
        self.assertFalse(leader_wanted(AppState(), False))
        self.assertFalse(leader_wanted(AppState(mode="fsc"), True))
        self.assertFalse(leader_wanted(AppState(stopped=True), True))
        self.assertFalse(leader_wanted(ALIGNING, True))

    def test_statuses(self):
        engaged = ArmFollowState(engaged=True)
        cases = (
            (NEAR, AppState(), False, HOLD, "no leader"),
            (NEAR, AppState(mode="fsc"), True, HOLD, "holding"),
            (NEAR, AppState(stopped=True), True, HOLD, "holding"),
            (NEAR, ALIGNING, True, HOLD, "holding"),
            (None, AppState(), True, HOLD, "leader fault"),
        )
        for leader, app, has_leader, pose, status in cases:
            with self.subTest(status=status, app=app):
                result = plan_arm(HOLD, leader, engaged, app, has_leader, 0.1)
                self.assertEqual(result, (pose, ArmFollowState(engaged=False), status))

    def test_torque_off_holds_the_commanded_pose_and_skips_the_leader(self):
        off = AppState(stopped=True, torque_off=True)
        for has_leader in (True, False):
            with self.subTest(has_leader=has_leader):
                result = plan_arm(HOLD, NEAR, ArmFollowState(engaged=True), off, has_leader, 0.1)
                self.assertEqual(result, (HOLD, ArmFollowState(engaged=False), "torque off"))

    def test_syncing_then_following(self):
        pose, follow, status = plan_arm(HOLD, FAR, ArmFollowState(), AppState(), True, 0.0)
        self.assertEqual((pose, follow.engaged, status), (HOLD, False, "syncing"))
        pose, follow, status = plan_arm(HOLD, NEAR, ArmFollowState(), AppState(), True, 0.0)
        self.assertEqual((pose, follow.engaged, status), (NEAR, True, "following"))


if __name__ == "__main__":
    unittest.main()
