"""src/robot/recording/episode_loop.py: The per-episode sequence of a pick_egg recording session.

For each attempt: start-pose gate, announcement, one recorded record_loop phase, zero base, an
unrecorded reset phase (skipped after the last attempt unless re-recording, as in the fork's
lerobot_record.py), then discard (left arrow) or save the episode. A gate timeout skips the
attempt; Esc or a stop during the gate ends the session; Ctrl+C discards the unsaved episode and
ends it the same way, so the caller always runs one shutdown. The LeRobot calls are passed in
as callables (lerobot_io.py builds them), so the sequence is tested with fakes.

`events` is the fork's shared mutable flag dict from init_keyboard_listener(); clearing its
flags here is the fork's own protocol (examples/lekiwi/record.py), not local state.
"""

from collections.abc import Callable, MutableMapping
from dataclasses import dataclass
import logging

from robot.recording.session import (
    SessionPlan,
    SessionState,
    after_recorded,
    after_rerecord,
    after_skipped,
    after_stop,
    is_finished,
    is_last_episode,
    recording_message,
    skip_message,
)

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class SessionIO:
    """The side effects one session needs, each a no-argument callable except `say`."""

    events: MutableMapping[str, bool]
    gate: Callable[[], str]  # "ready" | "timeout" | "stop"
    record_episode: Callable[[], None]  # record_loop with the dataset for episode_time_s
    reset: Callable[[], None]  # record_loop without the dataset for reset_time_s
    save_episode: Callable[[], None]
    discard_episode: Callable[[], None]
    zero_base: Callable[[], None]
    say: Callable[[str], None]
    now: Callable[[], float]


def _clear_flags(events: MutableMapping[str, bool]) -> None:
    """Drop exit_early / rerecord_episode pressed before an episode so they do not end it at once."""
    events["exit_early"] = False
    events["rerecord_episode"] = False


def _safe_zero(io: SessionIO) -> None:
    try:
        io.zero_base()
    except Exception:  # a failed stop must not hide the original problem; the host watchdog remains
        logger.exception("Could not send zero base velocities")


def _one_attempt(plan: SessionPlan, state: SessionState, io: SessionIO) -> SessionState:
    outcome = io.gate()
    if outcome == "stop":
        return after_stop(state)
    if outcome == "timeout":
        io.say(skip_message(state))
        return after_skipped(state)
    _clear_flags(io.events)
    io.say(recording_message(plan, state))
    started = io.now()
    io.record_episode()
    duration = io.now() - started
    _safe_zero(io)
    if not io.events["stop_recording"] and (not is_last_episode(plan, state) or io.events["rerecord_episode"]):
        io.say("Reset")
        io.reset()
        _safe_zero(io)
    if io.events["rerecord_episode"]:
        io.say("Re-record episode")
        _clear_flags(io.events)
        io.discard_episode()
        return after_rerecord(state)
    io.save_episode()
    return after_recorded(state, duration)


def run_session(plan: SessionPlan, io: SessionIO, state: SessionState = SessionState()) -> SessionState:
    """Run attempts until num_episodes attempts, Esc, or Ctrl+C; return the final state."""
    try:
        while not is_finished(plan, state):
            if io.events["stop_recording"]:
                return after_stop(state)
            state = _one_attempt(plan, state, io)
        return state
    except KeyboardInterrupt:
        logger.warning("Ctrl+C: stopping the session; the unsaved episode is discarded")
        _safe_zero(io)
        io.discard_episode()
        return after_stop(state)
