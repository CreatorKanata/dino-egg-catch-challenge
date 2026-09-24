"""tests/robot/test_mode_manager.py: Hardware-free checks of the mode manager rules.

Each KachiButton command is applied to a frozen AppState; the tests pin down every rule in the
spec (Stop and its latch, resume by Go Go!, mode toggle, Thx routing and FSC voice toggle,
ignore-while-action, notice expiry) as recorded in docs/spec/operating-modes.md. Auto Catch start
and outcomes are in test_mode_manager_catch.py, Auto Release in test_mode_manager_release.py.
"""

from dataclasses import replace
import unittest

from robot.auto_catch import CatchRequest, start_catch
from robot.config import ARM_KEYS, NOTICE_SECONDS
from robot.mode_manager import AppState, apply_command, expire_notice, manual_control_allowed, start_auto_catch
from robot.mode_manager import with_notice

FSC = AppState(mode="fsc")
POSE = {key: 0.0 for key in ARM_KEYS}


class StopTests(unittest.TestCase):
    def test_stop_resets_everything_from_any_state(self):
        busy = AppState(mode="fsc", action="auto_catch", voice_listening=True, notice="x", notice_until=1.0,
                        notice_level="warning", catch=start_catch(0.0, POSE, POSE))
        for start in (AppState(), busy):
            with self.subTest(start=start):
                result = apply_command(start, "stop", 10.0)
                self.assertEqual(result.state, AppState(notice="STOP", notice_until=10.0 + NOTICE_SECONDS,
                                                        stopped=True))
                self.assertTrue(result.stop_base)
                self.assertTrue(result.disengage_arm)


class ModeToggleTests(unittest.TestCase):
    def test_toggles_between_manual_and_fsc(self):
        to_fsc = apply_command(AppState(), "mode_toggle", 1.0)
        self.assertEqual((to_fsc.state.mode, to_fsc.state.notice), ("fsc", "FSC"))
        self.assertTrue(to_fsc.stop_base and to_fsc.disengage_arm)
        back = apply_command(replace(to_fsc.state, voice_listening=True), "mode_toggle", 2.0)
        self.assertEqual((back.state.mode, back.state.notice, back.state.voice_listening), ("manual", "MANUAL", False))
        self.assertTrue(back.stop_base and back.disengage_arm)
        self.assertEqual(back.state.notice_until, 2.0 + NOTICE_SECONDS)

    def test_ignored_while_action_runs(self):
        busy = AppState(action="auto_release")
        result = apply_command(busy, "mode_toggle", 1.0)
        self.assertEqual(result.state, busy)
        self.assertFalse(result.stop_base or result.disengage_arm)


class HiThxTests(unittest.TestCase):
    def test_hi_in_manual_needs_the_egg_size(self):
        result = apply_command(AppState(), "hi", 1.0)  # routed to start_auto_catch by the loop
        self.assertEqual(result.state, AppState())
        self.assertFalse(result.stop_base or result.disengage_arm)

    def test_thx_in_manual_needs_the_basket_and_data(self):
        result = apply_command(AppState(), "thx", 1.0)  # routed to start_auto_release by the loop
        self.assertEqual(result.state, AppState())
        self.assertFalse(result.stop_base or result.disengage_arm)

    def test_hi_and_thx_ignored_while_action_runs(self):
        busy = AppState(action="auto_catch")
        for command in ("hi", "thx"):
            with self.subTest(command=command):
                self.assertEqual(apply_command(busy, command, 1.0).state, busy)

    def test_hi_in_fsc_toggles_voice(self):
        first = apply_command(FSC, "hi", 1.0)
        self.assertEqual((first.state.voice_listening, first.state.notice), (True, "Listening..."))
        second = apply_command(first.state, "hi", 2.0)
        self.assertEqual((second.state.voice_listening, second.state.notice), (False, "Voice input ended"))
        self.assertFalse(first.stop_base or second.disengage_arm)

    def test_thx_in_fsc_does_nothing(self):
        result = apply_command(FSC, "thx", 1.0)
        self.assertEqual(result.state, FSC)
        self.assertFalse(result.stop_base or result.disengage_arm)


class StopLatchAndUnknownTests(unittest.TestCase):
    def test_unknown_command_leaves_state_unchanged(self):
        stopped = AppState(stopped=True)
        self.assertEqual(apply_command(stopped, "dance", 1.0).state, stopped)

    def test_stopped_ignores_hi_and_thx(self):
        stopped = apply_command(FSC, "stop", 1.0).state
        for command in ("hi", "thx"):
            with self.subTest(command=command):
                result = apply_command(stopped, command, 2.0)
                self.assertEqual(result.state, stopped)
                self.assertFalse(result.stop_base or result.disengage_arm)

    def test_go_go_resumes_manual_from_stopped_in_both_prior_modes(self):
        for prior in (AppState(), AppState(mode="fsc", voice_listening=True)):
            with self.subTest(prior=prior.mode):
                stopped = apply_command(prior, "stop", 1.0).state
                result = apply_command(stopped, "mode_toggle", 2.0)
                self.assertEqual(result.state, AppState(notice="MANUAL", notice_until=2.0 + NOTICE_SECONDS))
                self.assertTrue(result.stop_base and result.disengage_arm)

    def test_stop_while_stopped_stays_stopped(self):
        stopped = apply_command(AppState(), "stop", 1.0).state
        result = apply_command(stopped, "stop", 2.0)
        self.assertTrue(result.state.stopped)
        self.assertTrue(result.stop_base and result.disengage_arm)

    def test_does_not_mutate_input(self):
        start = AppState()
        apply_command(start, "stop", 1.0)
        self.assertEqual(start, AppState())


class NoticeAndControlTests(unittest.TestCase):
    def test_notice_expires_after_deadline(self):
        state = start_auto_catch(AppState(), CatchRequest(), 1.0).state
        self.assertEqual(expire_notice(state, 1.0 + NOTICE_SECONDS), state)
        cleared = expire_notice(state, 1.0 + NOTICE_SECONDS + 0.01)
        self.assertEqual((cleared.notice, cleared.notice_until, cleared.notice_level), ("", 0.0, "info"))

    def test_custom_notice_duration(self):
        self.assertEqual(apply_command(FSC, "hi", 1.0, notice_s=0.5).state.notice_until, 1.5)

    def test_with_notice_sets_level(self):
        state = with_notice(AppState(), "Captured", 2.0)
        self.assertEqual((state.notice, state.notice_until, state.notice_level),
                         ("Captured", 2.0 + NOTICE_SECONDS, "info"))
        self.assertEqual(with_notice(AppState(), "x", 0.0, 1.0, "warning").notice_level, "warning")

    def test_manual_control_only_in_unstopped_idle_manual_mode(self):
        self.assertTrue(manual_control_allowed(AppState()))
        self.assertFalse(manual_control_allowed(FSC))
        self.assertFalse(manual_control_allowed(AppState(stopped=True)))
        self.assertFalse(manual_control_allowed(AppState(action="auto_catch")))


if __name__ == "__main__":
    unittest.main()
