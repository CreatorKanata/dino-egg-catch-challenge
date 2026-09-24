"""tests/robot/test_mode_manager_release.py: Checks of the Auto Release start, outcomes, and Stop.

The mode manager and manual_mode.py are pure, so the `Thx` refusals (no basket, basket too far, no
home pose, no motion), the start into the basket alignment, the phase notices and terminal results,
Stop cancelling the action in every phase (base zero, arm held), presses ignored while it runs, and
the per-frame step (base action and arm pose replace driving and following) are verified directly.
"""

from dataclasses import replace
import unittest

from robot.auto_release import GRIPPER_KEY, ReleaseRequest, ReleaseState, start_release
from robot.config import ARM_KEYS, NOTICE_SECONDS
from robot.manual_mode import CommandResult, fold_commands, release_arm_status, step_auto_release
from robot.mode_manager import AppState, apply_command, manual_control_allowed, release_notice, start_auto_release
from robot.vision.basket_size import BasketDetection

HOME = {key: 0.0 for key in ARM_KEYS}
FRAMES = tuple({**HOME, GRIPPER_KEY: float(index)} for index in range(5))
READY = ReleaseRequest(size="ok", home=HOME, frames=FRAMES)
BASKET = BasketDetection(cx=0.2, cy=0.4, w=0.5, h=0.5, area_fraction=0.2, fill=0.6)
RUNNING = replace(AppState(), action="auto_release", release=start_release(0.0, HOME, FRAMES))


class StartTests(unittest.TestCase):
    def test_refusals_show_a_warning_and_do_not_start(self):
        cases = ((ReleaseRequest(), "Basket not in view"), (replace(READY, size="too_small"), "Basket too far"),
                 (replace(READY, home=None), "Home pose not recorded"),
                 (replace(READY, frames=None), "Release motion not recorded"),
                 (replace(READY, frames=()), "Release motion not recorded"))
        for request, notice in cases:
            with self.subTest(notice=notice):
                result = start_auto_release(AppState(), request, 1.0)
                self.assertEqual(result.state, AppState(notice=notice, notice_until=1.0 + NOTICE_SECONDS,
                                                        notice_level="warning"))
                self.assertFalse(result.stop_base or result.disengage_arm)

    def test_ready_starts_the_basket_alignment(self):
        result = start_auto_release(AppState(), READY, 2.0)
        self.assertEqual((result.state.action, result.state.release.phase), ("auto_release", "align"))
        self.assertEqual((result.state.notice, result.state.notice_level), ("Aligning to basket...", "info"))
        self.assertTrue(result.disengage_arm)
        self.assertFalse(manual_control_allowed(result.state))

    def test_ignored_in_fsc_while_stopped_or_busy(self):
        for state in (AppState(mode="fsc"), AppState(stopped=True), RUNNING, AppState(action="auto_catch")):
            with self.subTest(state=state.action):
                self.assertEqual(start_auto_release(state, READY, 1.0).state, state)

    def test_fold_routes_thx_in_manual_and_ignores_it_in_fsc(self):
        self.assertEqual(fold_commands(AppState(), ("thx",), 1.0).app.notice, "Basket not in view")
        self.assertEqual(fold_commands(AppState(), ("thx",), 1.0, release=READY).app.action, "auto_release")
        self.assertEqual(fold_commands(AppState(mode="fsc"), ("thx",), 1.0, release=READY).app, AppState(mode="fsc"))


class RunningTests(unittest.TestCase):
    def test_presses_ignored_and_stop_cancels_in_every_phase(self):
        for phase in ("align", "home", "play", "return_home"):
            busy = replace(RUNNING, release=replace(RUNNING.release, phase=phase))
            with self.subTest(phase=phase):
                for command in ("hi", "thx", "mode_toggle"):
                    self.assertEqual(apply_command(busy, command, 2.0).state, busy)
                stopped = apply_command(busy, "stop", 2.0)
                self.assertEqual((stopped.state.action, stopped.state.release, stopped.state.stopped),
                                 ("none", ReleaseState(), True))
                self.assertTrue(stopped.stop_base and stopped.disengage_arm)
                result, base, arm = step_auto_release(CommandResult(stopped.state, True, True), BASKET, HOME, 2.1)
                self.assertEqual((base, arm), (None, None))  # the loop then holds the arm and sends zeros

    def test_outcome_notices(self):
        cases = (("done", "Released!", "info"), ("lost", "Basket lost", "warning"),
                 ("align_timeout", "Could not align", "warning"), ("home_timeout", "Arm did not reach home", "warning"),
                 ("start_timeout", "Arm did not reach the release start", "warning"))
        for outcome, notice, level in cases:
            with self.subTest(outcome=outcome):
                ended = release_notice(RUNNING, outcome, 3.0)
                self.assertEqual(ended.state, AppState(notice=notice, notice_until=3.0 + NOTICE_SECONDS,
                                                       notice_level=level))
                self.assertTrue(ended.stop_base and ended.disengage_arm)
        playing = release_notice(RUNNING, "play_started", 3.0)
        self.assertEqual((playing.state.action, playing.state.notice), ("auto_release", "Releasing..."))
        self.assertFalse(playing.stop_base or playing.disengage_arm)
        self.assertEqual(release_notice(RUNNING, "running", 3.0).state, RUNNING)
        self.assertEqual(release_notice(AppState(), "done", 3.0).state, AppState())

    def test_step_returns_base_and_arm_while_running(self):
        result, base, arm = step_auto_release(CommandResult(RUNNING, False, False), BASKET, HOME, 0.1)
        self.assertEqual(result.app.action, "auto_release")
        self.assertGreater(base["y.vel"], 0.0)  # basket left of the target -> move left
        self.assertEqual(arm, HOME)
        self.assertEqual(release_arm_status(result.app), "auto release")
        self.assertEqual(release_arm_status(AppState()), "holding")
        idle = CommandResult(AppState(), False, False)
        self.assertEqual(step_auto_release(idle, BASKET, HOME, 0.1), (idle, None, None))

    def test_terminal_step_ends_the_action(self):
        lost = replace(RUNNING, release=replace(RUNNING.release, align=replace(RUNNING.release.align, lost_frames=99)))
        result, base, arm = step_auto_release(CommandResult(lost, False, False), None, HOME, 0.1)
        self.assertEqual((result.app.action, result.app.notice), ("none", "Basket lost"))
        self.assertTrue(result.stop_base and result.disengage_arm)
        self.assertEqual((base, arm), ({"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}, HOME))


if __name__ == "__main__":
    unittest.main()
