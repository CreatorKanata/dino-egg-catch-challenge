"""tests/robot/test_recording_speech.py: Checks of the recorder's voice choice and serialized speech.

Voice selection from sample `say -v '?'` output (Japanese default voices present, Samantha
present or absent, no English voice), `--voice none` and fixed voices, the `say -v` command
through a fake runner, one-at-a-time playback in order, gate-hint replacement and dropping, and
wait_idle. No real `say` process is started.
"""

import threading
import unittest

from robot.recording.speech import (
    Speaker,
    Voice,
    VoiceSetting,
    choose_voice,
    make_player,
    parse_voices,
    resolve_voice,
)

SAY_LIST = """\
Albert              en_US    # Hello! My name is Albert.
Bad News            en_US    # Hello! My name is Bad News.
Eddy (日本語（日本）)      ja_JP    # こんにちは! 私の名前はEddyです。
Kyoko               ja_JP    # こんにちは! 私の名前はKyokoです。
Karen               en_AU    # Hello! My name is Karen.
Samantha            en_US    # Hello! My name is Samantha.
"""
WITHOUT_SAMANTHA = "\n".join(line for line in SAY_LIST.splitlines() if not line.startswith("Samantha"))
ONLY_JAPANESE = "Kyoko               ja_JP    # こんにちは!\n"


class FakeRunner:
    def __init__(self, output="", code=0):
        self.output, self.code, self.commands = output, code, []

    def __call__(self, command):
        self.commands.append(list(command))
        return self.code, self.output


class VoiceTests(unittest.TestCase):
    def test_parse_keeps_names_with_spaces_and_parentheses(self):
        voices = parse_voices(SAY_LIST)
        self.assertIn(Voice("Bad News", "en_US"), voices)
        self.assertIn(Voice("Eddy (日本語（日本）)", "ja_JP"), voices)
        self.assertEqual(len(voices), 6)

    def test_preference_wins_over_list_order(self):
        self.assertEqual(choose_voice(parse_voices(SAY_LIST)), "Samantha")

    def test_next_preference_when_samantha_is_absent(self):
        self.assertEqual(choose_voice(parse_voices(WITHOUT_SAMANTHA)), "Karen")

    def test_first_english_voice_without_any_preference(self):
        self.assertEqual(choose_voice(parse_voices(WITHOUT_SAMANTHA), preference=("Ava",)), "Albert")

    def test_variant_name_matches_a_preference(self):
        self.assertEqual(choose_voice([Voice("Ava (Premium)", "en_US")]), "Ava (Premium)")

    def test_no_english_voice(self):
        self.assertIsNone(choose_voice(parse_voices(ONLY_JAPANESE)))

    def test_auto_lists_voices_once_on_macos(self):
        runner = FakeRunner(SAY_LIST)
        self.assertEqual(resolve_voice("auto", "Darwin", runner), VoiceSetting(True, "Samantha", "Samantha"))
        self.assertEqual(runner.commands, [["say", "-v", "?"]])

    def test_auto_without_english_or_when_listing_fails_uses_the_default(self):
        for runner in (FakeRunner(ONLY_JAPANESE), FakeRunner(SAY_LIST, code=1)):
            setting = resolve_voice("auto", "Darwin", runner)
            self.assertEqual((setting.enabled, setting.voice), (True, None))

    def test_none_fixed_and_other_platforms(self):
        self.assertFalse(resolve_voice("none", "Darwin", FakeRunner()).enabled)
        self.assertEqual(resolve_voice("Daniel", "Darwin", FakeRunner()).voice, "Daniel")
        self.assertIsNone(resolve_voice("auto", "Linux", FakeRunner()).voice)

    def test_player_runs_say_with_the_voice(self):
        runner = FakeRunner()
        make_player(VoiceSetting(True, "Samantha", "Samantha"), "Darwin", runner)("Reset")
        make_player(VoiceSetting(True, None, "default"), "Darwin", runner)("Reset")
        self.assertEqual(runner.commands, [["say", "-v", "Samantha", "Reset"], ["say", "Reset"]])

    def test_failed_say_is_logged(self):
        with self.assertLogs("robot.recording.speech", "WARNING"):
            make_player(VoiceSetting(True, "Nobody", "Nobody"), "Darwin", FakeRunner(code=1))("Reset")

    def test_voice_none_has_no_player_and_still_logs(self):
        self.assertIsNone(make_player(VoiceSetting(False, None, "none"), "Darwin", FakeRunner()))
        speaker = Speaker(None)
        with self.assertLogs("robot.recording.speech", "INFO") as logs:
            speaker.say("Recording episode 1 of 2, green egg")
        self.assertIn("Recording episode 1 of 2", logs.output[0])
        self.assertTrue(speaker.wait_idle(0.1))
        speaker.close(0.1)


class GatedPlayer:
    """Blocks inside the first utterance until released, and records overlap."""

    def __init__(self):
        self.played, self.active, self.overlap = [], 0, False
        self.started, self.release = threading.Event(), threading.Event()
        self.lock = threading.Lock()

    def __call__(self, text):
        with self.lock:
            self.active += 1
            self.overlap = self.overlap or self.active > 1
        self.started.set()
        self.release.wait(5)
        with self.lock:
            self.played.append(text)
            self.active -= 1


class SpeakerTests(unittest.TestCase):
    def speaker_busy_with(self, first):
        player = GatedPlayer()
        speaker = Speaker(player)
        self.addCleanup(speaker.close, 0.1)
        self.addCleanup(player.release.set)
        speaker.say(first)
        self.assertTrue(player.started.wait(5))
        return speaker, player

    def finish(self, speaker, player):
        player.release.set()
        self.assertTrue(speaker.wait_idle(5))
        self.assertFalse(player.overlap)
        return player.played

    def test_messages_play_one_at_a_time_in_order(self):
        speaker, player = self.speaker_busy_with("one")
        speaker.say("two")
        speaker.say("three")
        self.assertFalse(speaker.wait_idle(0.05))  # still playing "one"
        self.assertEqual(self.finish(speaker, player), ["one", "two", "three"])

    def test_newer_gate_hint_replaces_an_unplayed_one(self):
        speaker, player = self.speaker_busy_with("Reset")
        speaker.say_replaceable("wrist flex off by 40 degrees")
        speaker.say_replaceable("wrist flex off by 20 degrees")
        self.assertEqual(self.finish(speaker, player), ["Reset", "wrist flex off by 20 degrees"])

    def test_gate_hint_is_dropped_behind_a_queued_message(self):
        speaker, player = self.speaker_busy_with("hint 1")
        speaker.say("Recording episode 1 of 2, green egg")
        speaker.say_replaceable("hint 2")
        self.assertEqual(self.finish(speaker, player), ["hint 1", "Recording episode 1 of 2, green egg"])

    def test_normal_message_queues_behind_a_hint(self):
        speaker, player = self.speaker_busy_with("Reset")
        speaker.say_replaceable("hint")
        speaker.say("Recording episode 2 of 2, green egg")
        self.assertEqual(self.finish(speaker, player), ["Reset", "hint", "Recording episode 2 of 2, green egg"])

    def test_player_errors_do_not_stop_the_worker(self):
        played = []

        def flaky(text):
            if text == "bad":
                raise RuntimeError("say crashed")
            played.append(text)

        speaker = Speaker(flaky)
        with self.assertLogs("robot.recording.speech", "ERROR"):
            speaker.say("bad")
            speaker.say("good")
            self.assertTrue(speaker.wait_idle(5))
        speaker.close(1.0)
        self.assertEqual(played, ["good"])

    def test_close_drops_the_rest_and_ignores_later_messages(self):
        speaker, player = self.speaker_busy_with("one")
        speaker.say("two")
        speaker.close(0.05)
        speaker.say("three")
        player.release.set()
        self.assertNotIn("three", player.played)


if __name__ == "__main__":
    unittest.main()
