"""tests/robot/test_mode_manager_catch.py: Checks of the Auto Catch start, outcomes, and Stop (Phase 3 step 1).

The mode manager and manual_mode.py are pure, so the `Hi!` refusals (egg size, catch pose or
release pose not recorded), the start into the alignment, the notices of every outcome (terminal
or not), Stop cancelling the action in every phase (base zero, arm held), presses ignored while it
runs, and the per-frame step (base action, arm pose, and the alignment trace row) are verified
directly.
"""

from dataclasses import replace
import unittest

from robot.auto_catch import CatchRequest, CatchState, start_catch
from robot.config import ARM_KEYS, NOTICE_SECONDS
from robot.manual_mode import CommandResult, auto_arm_status, fold_commands, step_auto_catch
from robot.mode_manager import AppState, apply_command, catch_notice, manual_control_allowed, start_auto_catch
from robot.vision.egg_size import EggDetection

HOME = {key: 0.0 for key in ARM_KEYS}
CATCH = {key: 10.0 for key in ARM_KEYS}
READY = CatchRequest(size="ok", catch=CATCH, home=HOME, arm=HOME)  # the arm is at the release pose
FAR_EGG = EggDetection(cx=0.8, cy=0.5, w=0.2, h=0.3, color="green", spots=3, area_px=5000)
RUNNING = replace(AppState(), action="auto_catch", catch=start_catch(0.0, CATCH, HOME, HOME))
ZEROS = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}


class StartTests(unittest.TestCase):
    def test_refusals_show_a_warning_and_do_not_start(self):
        cases = ((CatchRequest(), "No egg in view"), (replace(READY, size="too_small"), "Egg too far"),
                 (replace(READY, size="too_large"), "Egg too close"),
                 (replace(READY, catch=None), "Catch pose not recorded"),
                 (replace(READY, home=None), "Home pose not recorded"))
        for request, notice in cases:
            with self.subTest(notice=notice):
                result = start_auto_catch(AppState(), request, 1.0)
                self.assertEqual(result.state, AppState(notice=notice, notice_until=1.0 + NOTICE_SECONDS,
                                                        notice_level="warning"))
                self.assertFalse(result.stop_base or result.disengage_arm)

    def test_ready_starts_the_alignment(self):
        result = start_auto_catch(AppState(), READY, 2.0)
        self.assertEqual((result.state.action, result.state.catch), ("auto_catch", start_catch(2.0, CATCH, HOME, HOME)))
        low = start_auto_catch(AppState(), replace(READY, arm={**HOME, "arm_wrist_flex.pos": 40.0}), 2.0)
        self.assertEqual((low.state.action, low.state.catch.phase), ("auto_catch", "to_start"))
        self.assertEqual((result.state.notice, result.state.notice_level), ("Aligning...", "info"))
        self.assertTrue(result.disengage_arm)
        self.assertFalse(manual_control_allowed(result.state))

    def test_ignored_in_fsc_while_stopped_or_busy(self):
        for state in (AppState(mode="fsc"), AppState(stopped=True), RUNNING, AppState(action="auto_release")):
            with self.subTest(state=state.action):
                self.assertEqual(start_auto_catch(state, READY, 1.0).state, state)

    def test_fold_routes_hi_in_manual_and_toggles_voice_in_fsc(self):
        self.assertEqual(fold_commands(AppState(), ("hi",), 1.0).app.notice, "No egg in view")
        self.assertEqual(fold_commands(AppState(), ("hi",), 1.0, catch=READY).app.action, "auto_catch")
        self.assertTrue(fold_commands(AppState(mode="fsc"), ("hi",), 1.0, catch=READY).app.voice_listening)


class RunningTests(unittest.TestCase):
    def test_presses_ignored_and_stop_cancels_in_every_phase(self):
        for phase in ("to_start", "align", "to_catch", "wrist_check", "pick_stub", "to_release"):
            busy = replace(RUNNING, catch=replace(RUNNING.catch, phase=phase))
            with self.subTest(phase=phase):
                for command in ("hi", "thx", "mode_toggle"):
                    self.assertEqual(apply_command(busy, command, 2.0).state, busy)
                stopped = apply_command(busy, "stop", 2.0)
                self.assertEqual((stopped.state.action, stopped.state.catch, stopped.state.stopped),
                                 ("none", CatchState(), True))
                self.assertTrue(stopped.stop_base and stopped.disengage_arm)
                result, base, arm, row = step_auto_catch(CommandResult(stopped.state, True, True), FAR_EGG, None,
                                                         HOME, 2.1)
                self.assertEqual((base, arm, row), (None, None, None))  # the loop holds the arm, sends zeros

    def test_outcome_notices(self):
        terminal = (("done", "Ready", "info"), ("lost", "Egg lost", "warning"),
                    ("align_timeout", "Could not align", "warning"),
                    ("release_timeout", "Arm did not reach the release pose", "warning"),
                    ("start_timeout", "Arm did not reach the release pose", "warning"),
                    ("catch_timeout_returned", "Arm did not reach the catch pose", "warning"),
                    ("no_wrist_egg_returned", "Egg not in wrist view", "warning"))
        for outcome, notice, level in terminal:
            with self.subTest(outcome=outcome):
                ended = catch_notice(RUNNING, outcome, 3.0)
                self.assertEqual(ended.state, AppState(notice=notice, notice_until=3.0 + NOTICE_SECONDS,
                                                       notice_level=level))
                self.assertTrue(ended.stop_base and ended.disengage_arm)
        ongoing = (("catch_timeout", "Arm did not reach the catch pose", "warning"),
                   ("no_wrist_egg", "Egg not in wrist view", "warning"),
                   ("policy_stub", "Catch: policy not available yet", "info"))
        for outcome, notice, level in ongoing:
            with self.subTest(outcome=outcome):
                shown = catch_notice(RUNNING, outcome, 3.0)
                self.assertEqual((shown.state.action, shown.state.notice, shown.state.notice_level),
                                 ("auto_catch", notice, level))
                self.assertFalse(shown.stop_base or shown.disengage_arm)
        self.assertEqual(catch_notice(RUNNING, "running", 3.0).state, RUNNING)
        self.assertEqual(catch_notice(AppState(), "done", 3.0).state, AppState())

    def test_step_returns_base_arm_and_trace_row_while_aligning(self):
        result, base, arm, row = step_auto_catch(CommandResult(RUNNING, False, False), FAR_EGG, None, HOME, 0.1)
        self.assertEqual((result.app.action, row.result, row.cx_raw), ("auto_catch", "running", FAR_EGG.cx))
        self.assertLess(base["y.vel"], 0.0)  # egg right of the target -> move right
        self.assertEqual(arm, HOME)
        self.assertEqual(auto_arm_status(result.app), "auto catch")
        self.assertEqual(auto_arm_status(replace(AppState(), action="auto_release")), "auto release")
        self.assertEqual(auto_arm_status(AppState()), "holding")
        idle = CommandResult(AppState(), False, False)
        self.assertEqual(step_auto_catch(idle, FAR_EGG, None, HOME, 0.1), (idle, None, None, None))

    def test_arm_phases_have_no_trace_row(self):
        moving = replace(RUNNING, catch=replace(RUNNING.catch, phase="to_catch"))
        result, base, arm, row = step_auto_catch(CommandResult(moving, False, False), FAR_EGG, None, HOME, 0.1)
        self.assertIsNone(row)
        self.assertEqual(base, ZEROS)
        self.assertGreater(arm["arm_elbow_flex.pos"], 0.0)  # moving toward the catch pose

    def test_terminal_step_ends_the_action(self):
        lost = replace(RUNNING, catch=replace(RUNNING.catch, align=replace(RUNNING.catch.align, lost_frames=99)))
        result, base, arm, row = step_auto_catch(CommandResult(lost, False, False), None, None, HOME, 0.1)
        self.assertEqual((result.app.action, result.app.notice, row.result), ("none", "Egg lost", "lost"))
        self.assertTrue(result.stop_base and result.disengage_arm)
        self.assertEqual((base, arm), (ZEROS, HOME))


if __name__ == "__main__":
    unittest.main()
