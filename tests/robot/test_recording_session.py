"""tests/robot/test_recording_session.py: Checks of the recorder's pure session planning and bookkeeping.

Covers SessionPlan from CLI arguments (defaults and overrides, validation), the task string with
the egg color, the recorded / skipped / re-record / stop transitions, the dataset root and
start checks, and the summary text. No LeRobot and no hardware.
"""

from argparse import Namespace
from pathlib import Path
import unittest

from robot.recording.config_recording import DEFAULT_NUM_EPISODES, DEFAULT_REPO_ID, DEFAULT_TASK
from robot.recording.record_pick_egg import parse_args
from robot.recording.session import (
    SessionPlan,
    SessionState,
    after_recorded,
    after_rerecord,
    after_skipped,
    after_stop,
    episode_number,
    is_finished,
    is_last_episode,
    plan_from_args,
    recording_message,
    resolve_root,
    skip_message,
    start_error,
    summary_lines,
    task_with_color,
)


def plan(**overrides):
    values = dict(repo_id="me/data", num_episodes=3, episode_time_s=20.0, reset_time_s=15.0, fps=30,
                  task="Pick up the egg with the mouth", color="green", push=True)
    return SessionPlan(**{**values, **overrides})


class PlanTests(unittest.TestCase):
    def test_defaults_from_parsed_args(self):
        result = plan_from_args(parse_args(["--egg-color", "green"]))
        self.assertEqual(result, SessionPlan(DEFAULT_REPO_ID, DEFAULT_NUM_EPISODES, 20.0, 15.0, 30,
                                             DEFAULT_TASK, "green", True))

    def test_overrides_from_parsed_args(self):
        args = parse_args(["--egg-color", "red", "--repo-id", "me/x", "--num-episodes", "5", "--episode-time-s",
                           "12", "--reset-time-s", "4.5", "--fps", "15", "--task", "Grab it", "--no-push"])
        self.assertEqual(plan_from_args(args), SessionPlan("me/x", 5, 12.0, 4.5, 15, "Grab it", "red", False))

    def test_invalid_values_are_rejected(self):
        base = vars(parse_args(["--egg-color", "green"]))
        for field, value in (("num_episodes", 0), ("episode_time_s", 0.0), ("reset_time_s", -1.0), ("fps", 0),
                             ("egg_color", "blue"), ("task", "  "), ("repo_id", "no-slash")):
            with self.subTest(field=field), self.assertRaises(ValueError):
                plan_from_args(Namespace(**{**base, field: value}))

    def test_zero_reset_time_is_allowed(self):
        base = vars(parse_args(["--egg-color", "green"]))
        self.assertEqual(plan_from_args(Namespace(**{**base, "reset_time_s": 0.0})).reset_time_s, 0.0)

    def test_task_with_color(self):
        self.assertEqual(task_with_color("Pick up the egg with the mouth", "red"),
                         "Pick up the egg with the mouth (red egg)")
        self.assertEqual(plan(color="yellow").single_task, "Pick up the egg with the mouth (yellow egg)")

    def test_plan_is_frozen(self):
        with self.assertRaises(AttributeError):
            plan().fps = 10  # type: ignore[misc]


class BookkeepingTests(unittest.TestCase):
    def test_recorded_advances_and_keeps_duration(self):
        state = after_recorded(SessionState(), 12.5)
        self.assertEqual((state.recorded, state.skipped, state.index, state.durations_s), (1, 0, 1, (12.5,)))

    def test_skipped_advances_without_recording(self):
        state = after_skipped(SessionState())
        self.assertEqual((state.recorded, state.skipped, state.index), (0, 1, 1))

    def test_rerecord_keeps_the_index(self):
        state = after_rerecord(after_recorded(SessionState(), 3.0))
        self.assertEqual((state.recorded, state.index, state.rerecorded), (1, 1, 1))

    def test_stop_finishes_the_session(self):
        state = after_stop(SessionState())
        self.assertTrue(state.stopped)
        self.assertTrue(is_finished(plan(), state))

    def test_transitions_do_not_mutate(self):
        start = SessionState()
        after_recorded(start, 1.0)
        after_skipped(start)
        after_stop(start)
        self.assertEqual(start, SessionState())

    def test_finished_after_all_attempts_including_skips(self):
        state = after_skipped(after_recorded(SessionState(), 1.0))
        self.assertFalse(is_finished(plan(), state))
        self.assertTrue(is_last_episode(plan(), state))
        self.assertEqual(episode_number(state), 3)
        self.assertTrue(is_finished(plan(), after_recorded(state, 2.0)))

    def test_messages(self):
        state = after_recorded(SessionState(), 1.0)
        self.assertEqual(recording_message(plan(), state), "Recording episode 2 of 3, green egg")
        self.assertIn("skipping episode 2", skip_message(state))


class RootAndSummaryTests(unittest.TestCase):
    def test_root_defaults_under_the_lerobot_home(self):
        self.assertEqual(resolve_root("me/data", None, Path("/home")), Path("/home/me/data"))
        self.assertEqual(resolve_root("me/data", "/tmp/x", Path("/home")), Path("/tmp/x"))

    def test_start_errors(self):
        self.assertIsNone(start_error(resume=False, root_exists=False, root=Path("/r")))
        self.assertIsNone(start_error(resume=True, root_exists=True, root=Path("/r")))
        self.assertIn("--resume", start_error(resume=False, root_exists=True, root=Path("/r")))
        self.assertIn("no dataset", start_error(resume=True, root_exists=False, root=Path("/r")))

    def test_summary_lists_counts_durations_root_and_push(self):
        state = after_rerecord(after_skipped(after_recorded(after_recorded(SessionState(), 10.0), 14.0)))
        text = "\n".join(summary_lines(plan(num_episodes=5), after_stop(state), Path("/r"), pushed=False))
        for expected in ("Recorded: 2 of 5", "Skipped: 1", "Re-recorded: 1", "10.0 s", "14.0 s", "mean 12.0 s",
                         "/r", "Pushed to the Hub: no", "stopped early"):
            self.assertIn(expected, text)

    def test_summary_completed_and_pushed(self):
        state = after_recorded(after_recorded(after_recorded(SessionState(), 1.0), 1.0), 1.0)
        text = "\n".join(summary_lines(plan(), state, Path("/r"), pushed=True))
        self.assertIn("Session completed", text)
        self.assertIn("Pushed to the Hub: yes (me/data)", text)

    def test_summary_without_episodes(self):
        text = "\n".join(summary_lines(plan(), SessionState(), Path("/r"), pushed=False))
        self.assertIn("Recorded: 0 of 3", text)
        self.assertNotIn("mean", text)


if __name__ == "__main__":
    unittest.main()
