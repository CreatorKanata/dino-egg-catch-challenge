"""src/robot/mode_manager.py: Phase 1 operating-mode state and KachiButton command rules.

Pure and stdlib-only. Implements the decisions in docs/spec/operating-modes.md, section 2:
`Stop` is a full stop and reset in every mode and latches a stopped state that only `Go Go!`
releases (back to Manual Mode); otherwise `Go Go!` toggles Manual Mode and FSC with zero base
velocities first, and presses are ignored while an automatic action runs. In Phase 1, Auto
Catch, Auto Release, and FSC are display-only stubs: no rule here ever starts motion.
"""

from dataclasses import dataclass, replace
from typing import Final, Literal

from robot.config import NOTICE_SECONDS

Mode = Literal["manual", "fsc"]
Action = Literal["none", "auto_catch", "auto_release"]
MODES: Final = ("manual", "fsc")
ACTIONS: Final = ("none", "auto_catch", "auto_release")

NOTICE_STOP: Final = "STOP"
NOTICE_MODE: Final = {"manual": "MANUAL", "fsc": "FSC"}
NOTICE_AUTO_CATCH_STUB: Final = "Auto Catch: not available yet"
NOTICE_AUTO_RELEASE_STUB: Final = "Auto Release: not available yet"
NOTICE_LISTENING: Final = "Listening..."
NOTICE_VOICE_ENDED: Final = "Voice input ended"


@dataclass(frozen=True)
class AppState:
    """Current mode, running action, FSC voice input, signboard notice, and the Stop flag."""

    mode: Mode = "manual"
    action: Action = "none"
    voice_listening: bool = False
    notice: str = ""
    notice_until: float = 0.0
    stopped: bool = False


@dataclass(frozen=True)
class Transition:
    """Result of one command: the next state and what the control loop must do this frame."""

    state: AppState
    stop_base: bool = False
    disengage_arm: bool = False


def _with_notice(state: AppState, notice: str, now: float, notice_s: float) -> AppState:
    return replace(state, notice=notice, notice_until=now + notice_s)


def _stop(now: float, notice_s: float) -> Transition:
    state = _with_notice(AppState(stopped=True), NOTICE_STOP, now, notice_s)
    return Transition(state=state, stop_base=True, disengage_arm=True)


def _toggle_mode(state: AppState, now: float, notice_s: float) -> Transition:
    mode: Mode = "fsc" if state.mode == "manual" else "manual"
    toggled = replace(state, mode=mode, voice_listening=False)
    return Transition(state=_with_notice(toggled, NOTICE_MODE[mode], now, notice_s), stop_base=True, disengage_arm=True)


def _resume(state: AppState, now: float, notice_s: float) -> Transition:
    """Release the Stop latch: always back to Manual Mode (never a toggle to FSC)."""
    resumed = replace(state, mode="manual", voice_listening=False, stopped=False)
    return Transition(state=_with_notice(resumed, NOTICE_MODE["manual"], now, notice_s), stop_base=True,
                      disengage_arm=True)


def _hi(state: AppState, now: float, notice_s: float) -> Transition:
    if state.mode == "manual":  # Phase 1 stub: Auto Catch does not start
        return Transition(state=_with_notice(state, NOTICE_AUTO_CATCH_STUB, now, notice_s))
    listening = not state.voice_listening
    notice = NOTICE_LISTENING if listening else NOTICE_VOICE_ENDED
    return Transition(state=_with_notice(replace(state, voice_listening=listening), notice, now, notice_s))


def _thx(state: AppState, now: float, notice_s: float) -> Transition:
    if state.mode == "manual":  # Phase 1 stub: Auto Release does not start
        return Transition(state=_with_notice(state, NOTICE_AUTO_RELEASE_STUB, now, notice_s))
    return Transition(state=state)  # owner decision: no action in FSC


_HANDLERS: Final = {"mode_toggle": _toggle_mode, "hi": _hi, "thx": _thx}


def apply_command(state: AppState, command: str, now: float, notice_s: float = NOTICE_SECONDS) -> Transition:
    """Apply one KachiButton command at time `now`; never mutates `state`.

    `stop` always works and latches `stopped`. While stopped, only `mode_toggle` does anything:
    it resumes Manual Mode. Other known commands are ignored while an action runs. Unknown
    commands leave the state unchanged.
    """
    if command == "stop":
        return _stop(now, notice_s)
    handler = _HANDLERS.get(command)
    if handler is None:
        return Transition(state=state)
    if state.stopped:
        return _resume(state, now, notice_s) if command == "mode_toggle" else Transition(state=state)
    if state.action != "none":  # a press while an automatic action runs is ignored
        return Transition(state=state)
    return handler(state, now, notice_s)


def expire_notice(state: AppState, now: float) -> AppState:
    """Clear the notice once `now` is past its deadline."""
    if state.notice and now > state.notice_until:
        return replace(state, notice="", notice_until=0.0)
    return state


def manual_control_allowed(state: AppState) -> bool:
    """The controller drives the base only in Manual Mode and never while the Stop latch holds;
    in FSC the base is zero (Phase 1)."""
    return state.mode == "manual" and not state.stopped
