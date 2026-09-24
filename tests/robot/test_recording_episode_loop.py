"""tests/robot/test_recording_episode_loop.py: Checks of the per-episode sequence with fake callables.

Gate, announcement, recorded phase, zero base, reset (not after the last attempt unless
re-recording), save or discard, gate timeouts as skips, Esc and gate stops, flags pressed during
the gate, and Ctrl+C discarding the unsaved episode. No LeRobot and no hardware.
"""

import unittest

from robot.recording.episode_loop import SessionIO, run_session
from robot.recording.session import SessionPlan, SessionState

PLAN = SessionPlan("me/data", 2, 20.0, 15.0, 30, "Pick up the egg with the mouth", "green", True)


class Harness:
    """Records the call order; `on_record` lets a test press keys during an episode."""

    def __init__(self, gates=(), on_record=None):
        self.log, self.spoken = [], []
        self.gates = list(gates)
        self.on_record = on_record or (lambda events, count: None)
        self.events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}
        self.time, self.records = 0.0, 0

    def gate(self):
        self.log.append("gate")
        return self.gates.pop(0) if self.gates else "ready"

    def record(self):
        self.log.append("record")
        self.records += 1
        self.time += 10.0
        self.on_record(self.events, self.records)

    def io(self):
        return SessionIO(events=self.events, gate=self.gate, record_episode=self.record,
                         reset=lambda: self.log.append("reset"), save_episode=lambda: self.log.append("save"),
                         discard_episode=lambda: self.log.append("discard"),
                         zero_base=lambda: self.log.append("zero"), say=self.spoken.append, now=lambda: self.time)


class EpisodeLoopTests(unittest.TestCase):
    def test_two_episodes_with_a_reset_only_between_them(self):
        harness = Harness()
        state = run_session(PLAN, harness.io())
        self.assertEqual(harness.log, ["gate", "record", "zero", "reset", "zero", "save",
                                       "gate", "record", "zero", "save"])
        self.assertEqual(state, SessionState(recorded=2, index=2, durations_s=(10.0, 10.0)))
        self.assertEqual(harness.spoken, ["Recording episode 1 of 2, green egg", "Reset",
                                          "Recording episode 2 of 2, green egg"])

    def test_gate_timeout_skips_the_attempt(self):
        harness = Harness(gates=["timeout"])
        state = run_session(PLAN, harness.io())
        self.assertEqual((state.recorded, state.skipped, state.index), (1, 1, 2))
        self.assertIn("skipping episode 1", harness.spoken[0])
        self.assertEqual(harness.log[:2], ["gate", "gate"])

    def test_gate_stop_ends_without_recording(self):
        harness = Harness(gates=["stop"])
        state = run_session(PLAN, harness.io())
        self.assertTrue(state.stopped)
        self.assertEqual(harness.log, ["gate"])

    def test_rerecord_discards_and_repeats_the_attempt(self):
        def press_left_once(events, count):
            if count == 1:
                events["rerecord_episode"] = events["exit_early"] = True

        harness = Harness(on_record=press_left_once)
        state = run_session(PLAN, harness.io())
        self.assertEqual(harness.log[:6], ["gate", "record", "zero", "reset", "zero", "discard"])
        self.assertEqual((state.recorded, state.rerecorded, state.index), (2, 1, 2))
        self.assertFalse(harness.events["rerecord_episode"])

    def test_rerecord_on_the_last_attempt_still_resets(self):
        def press_left_on_second(events, count):
            if count == 2:
                events["rerecord_episode"] = True

        harness = Harness(on_record=press_left_on_second)
        run_session(PLAN, harness.io())
        self.assertEqual(harness.log[6:12], ["gate", "record", "zero", "reset", "zero", "discard"])

    def test_esc_saves_the_episode_and_stops(self):
        def press_esc(events, count):
            events["stop_recording"] = events["exit_early"] = True

        harness = Harness(on_record=press_esc)
        state = run_session(PLAN, harness.io())
        self.assertEqual(harness.log, ["gate", "record", "zero", "save"])
        self.assertEqual((state.recorded, state.stopped), (1, True))

    def test_keys_pressed_during_the_gate_are_cleared_before_recording(self):
        harness = Harness()
        harness.events.update(exit_early=True, rerecord_episode=True)
        run_session(SessionPlan(**{**PLAN.__dict__, "num_episodes": 1}), harness.io())
        self.assertEqual(harness.log, ["gate", "record", "zero", "save"])

    def test_ctrl_c_discards_and_stops(self):
        def interrupt(events, count):
            raise KeyboardInterrupt

        harness = Harness(on_record=interrupt)
        state = run_session(PLAN, harness.io())
        self.assertEqual(harness.log, ["gate", "record", "zero", "discard"])
        self.assertEqual((state.recorded, state.stopped), (0, True))

    def test_failed_zero_does_not_stop_the_session(self):
        harness = Harness()
        io = SessionIO(**{**harness.io().__dict__, "zero_base": lambda: 1 / 0})
        with self.assertLogs("robot.recording.episode_loop", "ERROR"):
            self.assertEqual(run_session(PLAN, io).recorded, 2)

    def test_waits_for_quiet_speech_between_the_announcement_and_recording(self):
        harness = Harness()
        io = SessionIO(**{**harness.io().__dict__, "wait_quiet": lambda: harness.log.append("quiet")})
        run_session(SessionPlan(**{**PLAN.__dict__, "num_episodes": 1}), io)
        self.assertEqual(harness.log, ["gate", "quiet", "record", "zero", "save"])

    def test_stop_already_set_ends_at_once(self):
        harness = Harness()
        harness.events["stop_recording"] = True
        self.assertTrue(run_session(PLAN, harness.io()).stopped)
        self.assertEqual(harness.log, [])


if __name__ == "__main__":
    unittest.main()
