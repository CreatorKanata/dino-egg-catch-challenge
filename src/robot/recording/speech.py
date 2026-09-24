"""src/robot/recording/speech.py: English, one-at-a-time spoken announcements for the pick_egg recorder.

Owner trial run (2026-09-25): the fork's log_say ran macOS `say` without a voice, so English came
out in the Mac's default Japanese voice (LANG/LC_ALL do not change it), and it does not wait, so two
announcements overlapped. Here macOS `say -v <voice>` runs on one worker thread that waits for each
utterance to finish. "auto" picks the first installed voice of RECORDER_VOICE_PREFERENCE, else the
first `en_` voice (from `say -v '?'`). Gate guidance is replaceable: a newer hint replaces an unplayed
one and is dropped behind other queued messages. Other platforms use the fork's `say` (blocking).
Stdlib-only (LeRobot imported lazily off macOS); the command runner is injectable for tests.
"""

from collections import deque
from collections.abc import Callable, Sequence
from dataclasses import dataclass
import logging
import platform
import re
import subprocess
import threading
import time

from robot.recording.config_recording import RECORDER_VOICE_PREFERENCE

logger = logging.getLogger(__name__)

# `Samantha            en_US    # Hello! ...`; names may contain spaces and parentheses.
_VOICE_LINE = re.compile(r"^(?P<name>.+?)\s+(?P<lang>[a-z]{2,3}_[A-Za-z0-9]+)\s+#")

Runner = Callable[[Sequence[str]], tuple[int, str]]  # command -> (return code, stdout)


@dataclass(frozen=True)
class Voice:
    name: str
    language: str


@dataclass(frozen=True)
class VoiceSetting:
    """Whether to speak, the `say -v` voice (None: system default), and a label for the start-up print."""

    enabled: bool
    voice: str | None
    label: str


def run_command(command: Sequence[str]) -> tuple[int, str]:
    """Run a command to completion; return its exit code and stdout."""
    result = subprocess.run(list(command), capture_output=True, text=True, check=False)
    return result.returncode, result.stdout


def parse_voices(output: str) -> tuple[Voice, ...]:
    """The voices listed by `say -v '?'`, in order."""
    matches = (_VOICE_LINE.match(line) for line in output.splitlines())
    return tuple(Voice(m.group("name").strip(), m.group("lang")) for m in matches if m)


def choose_voice(voices: Sequence[Voice], preference: Sequence[str] = RECORDER_VOICE_PREFERENCE) -> str | None:
    """First installed preferred voice (exact name or `<name> (...)`), else the first `en_` voice, else None."""
    for wanted in preference:
        for voice in voices:
            if voice.name == wanted or voice.name.startswith(f"{wanted} ("):
                return voice.name
    return next((voice.name for voice in voices if voice.language.startswith("en_")), None)


def resolve_voice(setting: str, system: str | None = None, runner: Runner = run_command,
                  preference: Sequence[str] = RECORDER_VOICE_PREFERENCE) -> VoiceSetting:
    """Turn the `--voice` value into a VoiceSetting; "auto" lists the macOS voices once."""
    system = platform.system() if system is None else system
    if setting.lower() == "none":
        return VoiceSetting(False, None, "none (speech off; announcements are only logged)")
    if system != "Darwin":
        return VoiceSetting(True, None, f"LeRobot say on {system} (voice not selectable)")
    if setting.lower() != "auto":
        return VoiceSetting(True, setting, setting)
    try:
        code, output = runner(["say", "-v", "?"])
    except OSError as error:
        logger.warning("Cannot list the macOS voices: %s", error)
        code, output = 1, ""
    voice = choose_voice(parse_voices(output), preference) if code == 0 else None
    return VoiceSetting(True, voice, voice or "system default (no English voice found)")


def make_player(setting: VoiceSetting, system: str | None = None,
                runner: Runner = run_command) -> Callable[[str], None] | None:
    """A blocking `text -> speech` function, or None when speech is off."""
    if not setting.enabled:
        return None
    if (platform.system() if system is None else system) == "Darwin":
        base = ("say", "-v", setting.voice) if setting.voice else ("say",)

        def play_macos(text: str) -> None:
            code, _ = runner([*base, text])
            if code != 0:
                logger.warning("`say` exited with %d for: %s", code, text)

        return play_macos

    def play_fork(text: str) -> None:
        from lerobot.utils.utils import say

        say(text, blocking=True)

    return play_fork


class Speaker:
    """Logs every announcement and plays them one at a time on a daemon worker thread.

    The pending queue is the worker's shared state (guarded by a Condition); nothing else mutates.
    """

    def __init__(self, play: Callable[[str], None] | None) -> None:
        self._play = play
        self._pending: deque[tuple[str, bool]] = deque()
        self._busy = False
        self._closed = False
        self._cond = threading.Condition()
        self._thread = None
        if play is not None:
            self._thread = threading.Thread(target=self._run, name="recorder-speech", daemon=True)
            self._thread.start()

    def say(self, text: str) -> None:
        """Queue an episode or session message; it always plays, in order."""
        self._enqueue(text, replaceable=False)

    def say_replaceable(self, text: str) -> None:
        """Queue gate guidance: replaces an unplayed hint, dropped behind other queued messages."""
        self._enqueue(text, replaceable=True)

    def _enqueue(self, text: str, replaceable: bool) -> None:
        logger.info("Say: %s", text)
        if self._play is None:
            return
        with self._cond:
            if self._closed:
                return
            if replaceable and self._pending:
                if not self._pending[-1][1]:
                    return  # a different message is waiting; this hint would be stale by then
                self._pending.pop()
            self._pending.append((text, replaceable))
            self._cond.notify_all()

    def _run(self) -> None:
        while True:
            with self._cond:
                while not self._pending and not self._closed:
                    self._cond.wait()
                if not self._pending:
                    return
                text, _ = self._pending.popleft()
                self._busy = True
            try:
                self._play(text)
            except Exception:
                logger.exception("Speech failed for: %s", text)
            finally:
                with self._cond:
                    self._busy = False
                    self._cond.notify_all()

    def wait_idle(self, timeout_s: float) -> bool:
        """Wait until nothing is queued or playing, at most `timeout_s`; True when idle."""
        deadline = time.monotonic() + timeout_s
        with self._cond:
            while self._pending or self._busy:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._cond.wait(remaining)
            return True

    def close(self, timeout_s: float) -> None:
        """Let queued speech play for at most `timeout_s`, then drop the rest and stop the worker."""
        if self._thread is None:
            return
        self.wait_idle(timeout_s)
        with self._cond:
            self._closed = True
            self._pending.clear()
            self._cond.notify_all()
        self._thread.join(timeout=1.0)
