"""src/robot/kachi_phrases.py: Turn KachiButton keystrokes into mode-manager commands.

The KachiButtons type plain ASCII phrases into whichever window has keyboard focus: the attendee
unit "Go Go!", "Hi!", "Thx"; the stop unit "STOP" / "Stop" (stop), "OFF" / "Off" (arm torque off),
and "MODE" / "Mode" (back to Manual Mode), both spellings because its typed text is unconfirmed
(config.KACHI_PHRASES); the signboard window keeps focus for the whole exhibit (owner decision in
docs/spec/operating-modes.md). This pure, stdlib-only module matches typed chunks against complete
phrases. The caller passes the time, so there are no timers and the rules are unit-tested.
"""

from collections.abc import Sequence
from dataclasses import dataclass

from robot.config import KACHI_BUFFER_MAX, KACHI_PHRASE_GAP_S, KACHI_PHRASES


@dataclass(frozen=True)
class PhraseBuffer:
    """Recently typed characters that may still complete a phrase, and when they arrived."""

    text: str = ""
    last_char_time: float | None = None


def _match(text: str, phrases: Sequence[tuple[str, str]]) -> str | None:
    """The command whose phrase `text` ends with, or None."""
    return next((command for phrase, command in phrases if text.endswith(phrase)), None)


def _scan(text: str, phrases: Sequence[tuple[str, str]]) -> tuple[str, tuple[str, ...]]:
    """Walk `text` character by character; emit a command and restart after each full phrase."""
    pending, commands = "", ()
    for char in text:
        pending = pending + char
        command = _match(pending, phrases)
        if command is not None:
            pending, commands = "", commands + (command,)
    return pending, commands


def feed(
    buffer: PhraseBuffer,
    text: str,
    now: float,
    phrases: Sequence[tuple[str, str]] = KACHI_PHRASES,
    gap_s: float = KACHI_PHRASE_GAP_S,
    max_len: int = KACHI_BUFFER_MAX,
) -> tuple[PhraseBuffer, tuple[str, ...]]:
    """Append typed `text` at time `now`; return the new buffer and the commands it completed.

    A pause longer than `gap_s` since the previous character discards the partial phrase. Every
    completed phrase emits its command and clears the buffer, so "Hi!Hi!" yields two commands.
    Only the last `max_len` characters are kept. The input buffer is never modified.
    """
    if not text:
        return buffer, ()
    expired = buffer.last_char_time is not None and now - buffer.last_char_time > gap_s
    pending, commands = _scan(("" if expired else buffer.text) + text, phrases)
    return PhraseBuffer(text=pending[-max_len:] if max_len > 0 else "", last_char_time=now), commands
