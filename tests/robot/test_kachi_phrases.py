"""tests/robot/test_kachi_phrases.py: Hardware-free checks of KachiButton phrase detection.

The KachiButton types plain ASCII phrases into the focused signboard window; kachi_phrases.feed
turns typed chunks into commands. These tests cover chunking, the 1 s gap rule, back-to-back
phrases, exact matching, and the buffer cap without pygame or a keyboard.
"""

import unittest

from robot.config import KACHI_BUFFER_MAX
from robot.kachi_phrases import PhraseBuffer, feed


def feed_all(chunks, start=PhraseBuffer()):
    """Feed (text, time) chunks in order; return the final buffer and all emitted commands."""
    buffer, commands = start, ()
    for text, now in chunks:
        buffer, emitted = feed(buffer, text, now)
        commands = commands + emitted
    return buffer, commands


class FeedTests(unittest.TestCase):
    def test_single_phrase_in_one_chunk(self):
        buffer, commands = feed(PhraseBuffer(), "Hi!", 0.0)
        self.assertEqual(commands, ("hi",))
        self.assertEqual(buffer.text, "")

    def test_all_phrases_map_to_commands(self):
        cases = {"Go Go!": "mode_toggle", "Hi!": "hi", "Thx": "thx", "Stop": "stop"}
        for phrase, command in cases.items():
            with self.subTest(phrase=phrase):
                self.assertEqual(feed(PhraseBuffer(), phrase, 5.0)[1], (command,))

    def test_phrase_split_across_chunks(self):
        chunks = [("G", 0.0), ("o", 0.05), (" ", 0.1), ("Go", 0.15), ("!", 0.2)]
        self.assertEqual(feed_all(chunks)[1], ("mode_toggle",))

    def test_gap_longer_than_limit_discards_partial(self):
        buffer, commands = feed_all([("St", 0.0), ("op", 1.5)])
        self.assertEqual(commands, ())
        self.assertEqual(buffer.text, "op")

    def test_gap_at_limit_keeps_partial(self):
        self.assertEqual(feed_all([("St", 0.0), ("op", 1.0)])[1], ("stop",))

    def test_two_phrases_back_to_back(self):
        self.assertEqual(feed(PhraseBuffer(), "Hi!Hi!", 0.0)[1], ("hi", "hi"))
        self.assertEqual(feed_all([("Hi!", 0.0), ("Stop", 0.1)])[1], ("hi", "stop"))

    def test_phrase_followed_by_more_text_in_one_chunk(self):
        buffer, commands = feed(PhraseBuffer(), "Thx.St", 0.0)
        self.assertEqual((commands, buffer.text), (("thx",), ".St"))

    def test_leading_noise_before_phrase_still_matches(self):
        self.assertEqual(feed(PhraseBuffer(), "xxStop", 0.0)[1], ("stop",))

    def test_unknown_text_never_matches(self):
        buffer, commands = feed_all([("hello world", 0.0), ("Hi", 0.1), ("Th", 0.2)])
        self.assertEqual(commands, ())
        self.assertTrue(buffer.text.endswith("HiTh"))

    def test_exact_case_space_and_punctuation(self):
        for text in ("stop", "STOP", "GoGo!", "Go Go", "go go!", "Hi", "hi!", "thx", "Th x", "S top"):
            with self.subTest(text=text):
                self.assertEqual(feed(PhraseBuffer(), text, 0.0)[1], ())

    def test_buffer_is_capped(self):
        buffer, _ = feed(PhraseBuffer(), "x" * (KACHI_BUFFER_MAX * 3), 0.0)
        self.assertEqual(len(buffer.text), KACHI_BUFFER_MAX)
        buffer, _ = feed(PhraseBuffer(), "abcdef", 0.0, max_len=4)
        self.assertEqual(buffer.text, "cdef")

    def test_does_not_mutate_input_and_records_time(self):
        start = PhraseBuffer(text="St", last_char_time=1.0)
        buffer, _ = feed(start, "o", 1.2)
        self.assertEqual(start, PhraseBuffer(text="St", last_char_time=1.0))
        self.assertEqual(buffer, PhraseBuffer(text="Sto", last_char_time=1.2))

    def test_empty_text_changes_nothing(self):
        start = PhraseBuffer(text="St", last_char_time=1.0)
        self.assertEqual(feed(start, "", 9.0), (start, ()))

    def test_custom_phrases(self):
        self.assertEqual(feed(PhraseBuffer(), "ab", 0.0, phrases=(("ab", "x"),))[1], ("x",))


if __name__ == "__main__":
    unittest.main()
