"""tests/robot/test_manual_mode.py: Hardware-free checks of the per-frame Manual Mode decisions.

manual_mode.py is stdlib-only, so command folding, the base decision (driving only in Manual
Mode and never in a stop frame), and the arm decision (no leader, held, fault, syncing,
following) are verified directly, independent of the loop wiring in test_drive_loop.py.
"""

from dataclasses import replace
import unittest

from robot.arm_follow import ArmFollowState
from robot.config import ARM_KEYS, NOTICE_SECONDS
from robot.dino_controller_reader import INITIAL_STATE
from robot.drive_state import DriveState
from robot.manual_mode import fold_commands, leader_wanted, plan_arm, plan_base
from robot.mode_manager import AppState

FORWARD = replace(INITIAL_STATE, synchronized=True, up=True)
DRIVE = DriveState(input_lost=False, pending_rotation_deg=10.0)
ZEROS = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
HOLD = {key: 0.0 for key in ARM_KEYS}
NEAR = {key: 1.0 for key in ARM_KEYS}
FAR = {key: 90.0 for key in ARM_KEYS}


class FoldCommandsTests(unittest.TestCase):
    def test_no_commands_only_expires_notice(self):
        noticed = AppState(notice="STOP", notice_until=1.0)
        result = fold_commands(noticed, (), 2.0)
        self.assertEqual((result.app.notice, result.stop_base, result.disengage_arm), ("", False, False))

    def test_commands_apply_in_order_and_flags_accumulate(self):
        result = fold_commands(AppState(), ("stop", "hi"), 5.0)
        self.assertTrue(result.stop_base and result.disengage_arm)
        self.assertTrue(result.app.stopped)  # "hi" is ignored while stopped
        resumed = fold_commands(AppState(), ("stop", "mode_toggle"), 5.0)
        self.assertEqual((resumed.app.stopped, resumed.app.mode, resumed.app.notice), (False, "manual", "MANUAL"))
        self.assertEqual(resumed.app.notice_until, 5.0 + NOTICE_SECONDS)
        self.assertEqual(fold_commands(AppState(), ("mode_toggle", "mode_toggle"), 0.0).app.mode, "manual")


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


class PlanArmTests(unittest.TestCase):
    def test_leader_wanted_only_in_unstopped_manual_mode(self):
        self.assertTrue(leader_wanted(AppState(), True))
        self.assertFalse(leader_wanted(AppState(), False))
        self.assertFalse(leader_wanted(AppState(mode="fsc"), True))
        self.assertFalse(leader_wanted(AppState(stopped=True), True))

    def test_statuses(self):
        engaged = ArmFollowState(engaged=True)
        cases = (
            (NEAR, AppState(), False, HOLD, "no leader"),
            (NEAR, AppState(mode="fsc"), True, HOLD, "holding"),
            (NEAR, AppState(stopped=True), True, HOLD, "holding"),
            (None, AppState(), True, HOLD, "leader fault"),
        )
        for leader, app, has_leader, pose, status in cases:
            with self.subTest(status=status, app=app):
                result = plan_arm(HOLD, leader, engaged, app, has_leader, 0.1)
                self.assertEqual(result, (pose, ArmFollowState(engaged=False), status))

    def test_syncing_then_following(self):
        pose, follow, status = plan_arm(HOLD, FAR, ArmFollowState(), AppState(), True, 0.0)
        self.assertEqual((pose, follow.engaged, status), (HOLD, False, "syncing"))
        pose, follow, status = plan_arm(HOLD, NEAR, ArmFollowState(), AppState(), True, 0.0)
        self.assertEqual((pose, follow.engaged, status), (NEAR, True, "following"))


if __name__ == "__main__":
    unittest.main()
