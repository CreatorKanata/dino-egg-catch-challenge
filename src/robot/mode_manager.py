"""src/robot/mode_manager.py: Operating-mode state and KachiButton command rules.

Pure and stdlib-only. Implements the decisions in docs/spec/operating-modes.md, section 2:
`Stop` is a full stop and reset in every mode and latches a stopped state that only `Go Go!`
releases (back to Manual Mode); otherwise `Go Go!` toggles Manual Mode and FSC with zero base
velocities first, and presses are ignored while an automatic action runs. Phase 2 step 1: `Hi!`
in Manual Mode checks the egg size (start_auto_catch) and starts the base alignment; the catch
itself, Auto Release, and FSC are still display-only stubs.
"""

from dataclasses import dataclass, replace
from typing import Final, Literal

from robot.align import AlignResult, AlignState, start_align
from robot.config import NOTICE_SECONDS
from robot.vision.egg_size import SizeClass

Mode = Literal["manual", "fsc"]
Action = Literal["none", "auto_catch", "auto_release"]
MODES: Final = ("manual", "fsc")
ACTIONS: Final = ("none", "auto_catch", "auto_release")

NOTICE_STOP: Final = "STOP"
NOTICE_MODE: Final = {"manual": "MANUAL", "fsc": "FSC"}
NOTICE_AUTO_RELEASE_STUB: Final = "Auto Release: not available yet"
NOTICE_LISTENING: Final = "Listening..."
NOTICE_VOICE_ENDED: Final = "Voice input ended"
NOTICE_SIZE: Final = {"none": "No egg in view", "too_small": "Egg too far", "too_large": "Egg too close"}
NOTICE_ALIGNING: Final = "Aligning..."
NOTICE_ALIGN_RESULT: Final = {
    "done": ("Aligned. Catch: not available yet", "info"),
    "lost": ("Egg lost", "warning"),
    "timeout": ("Could not align", "warning"),
}
NOTICE_CAPTURED: Final = "Captured"
NOTICE_CAPTURE_FAILED: Final = "Capture failed"

NoticeLevel = Literal["info", "warning"]
NOTICE_LEVELS: Final = ("info", "warning")


@dataclass(frozen=True)
class AppState:
    """Current mode, running action and its alignment, FSC voice input, notice, and the Stop flag."""

    mode: Mode = "manual"
    action: Action = "none"
    voice_listening: bool = False
    notice: str = ""
    notice_until: float = 0.0
    stopped: bool = False
    notice_level: NoticeLevel = "info"
    align: AlignState = AlignState()


@dataclass(frozen=True)
class Transition:
    """Result of one command: the next state and what the control loop must do this frame."""

    state: AppState
    stop_base: bool = False
    disengage_arm: bool = False


def with_notice(
    state: AppState, notice: str, now: float, notice_s: float = NOTICE_SECONDS, level: NoticeLevel = "info"
) -> AppState:
    """`state` showing `notice` (info or warning color) until now + notice_s."""
    return replace(state, notice=notice, notice_until=now + notice_s, notice_level=level)


def _stop(now: float, notice_s: float) -> Transition:
    state = with_notice(AppState(stopped=True), NOTICE_STOP, now, notice_s)
    return Transition(state=state, stop_base=True, disengage_arm=True)


def _toggle_mode(state: AppState, now: float, notice_s: float) -> Transition:
    mode: Mode = "fsc" if state.mode == "manual" else "manual"
    toggled = replace(state, mode=mode, voice_listening=False)
    return Transition(state=with_notice(toggled, NOTICE_MODE[mode], now, notice_s), stop_base=True, disengage_arm=True)


def _resume(state: AppState, now: float, notice_s: float) -> Transition:
    """Release the Stop latch: always back to Manual Mode (never a toggle to FSC)."""
    resumed = replace(state, mode="manual", voice_listening=False, stopped=False)
    return Transition(state=with_notice(resumed, NOTICE_MODE["manual"], now, notice_s), stop_base=True,
                      disengage_arm=True)


def _hi(state: AppState, now: float, notice_s: float) -> Transition:
    if state.mode == "manual":  # needs the egg size: the loop routes it to start_auto_catch
        return Transition(state=state)
    listening = not state.voice_listening
    notice = NOTICE_LISTENING if listening else NOTICE_VOICE_ENDED
    return Transition(state=with_notice(replace(state, voice_listening=listening), notice, now, notice_s))


def _thx(state: AppState, now: float, notice_s: float) -> Transition:
    if state.mode == "manual":  # Phase 1 stub: Auto Release does not start
        return Transition(state=with_notice(state, NOTICE_AUTO_RELEASE_STUB, now, notice_s))
    return Transition(state=state)  # owner decision: no action in FSC


_HANDLERS: Final = {"mode_toggle": _toggle_mode, "hi": _hi, "thx": _thx}


def apply_command(state: AppState, command: str, now: float, notice_s: float = NOTICE_SECONDS) -> Transition:
    """Apply one KachiButton command at time `now`; never mutates `state`.

    `stop` always works, latches `stopped`, and cancels any action. While stopped, only
    `mode_toggle` does anything: it resumes Manual Mode. Other known commands are ignored while
    an action runs. Unknown commands leave the state unchanged. `hi` in Manual Mode is left
    unchanged here because it needs the egg size; use start_auto_catch for it.
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


def start_auto_catch(state: AppState, size: SizeClass, now: float, notice_s: float = NOTICE_SECONDS) -> Transition:
    """`Hi!` in Manual Mode with the latest front-camera size class.

    Anything but "ok" shows a warning notice and nothing moves; "ok" starts the alignment. Only
    in Manual Mode, not stopped, and with no action running; otherwise the state is unchanged.
    """
    if state.mode != "manual" or state.stopped or state.action != "none":
        return Transition(state=state)
    if size != "ok":
        return Transition(state=with_notice(state, NOTICE_SIZE[size], now, notice_s, "warning"))
    started = replace(state, action="auto_catch", align=start_align(now))
    return Transition(state=with_notice(started, NOTICE_ALIGNING, now, notice_s), disengage_arm=True)


def finish_auto_catch(
    state: AppState, result: AlignResult, now: float, notice_s: float = NOTICE_SECONDS
) -> Transition:
    """End the alignment with a terminal result: back to Manual Mode with zero base velocities
    this frame and arm following disengaged (it re-syncs slowly). "running" changes nothing."""
    if result not in NOTICE_ALIGN_RESULT or state.action != "auto_catch":
        return Transition(state=state)
    notice, level = NOTICE_ALIGN_RESULT[result]
    ended = replace(state, action="none", align=AlignState())
    return Transition(state=with_notice(ended, notice, now, notice_s, level), stop_base=True, disengage_arm=True)


def expire_notice(state: AppState, now: float) -> AppState:
    """Clear the notice once `now` is past its deadline."""
    if state.notice and now > state.notice_until:
        return replace(state, notice="", notice_until=0.0, notice_level="info")
    return state


def manual_control_allowed(state: AppState) -> bool:
    """The controller and the leader arm drive only in Manual Mode, never while the Stop latch
    holds, and never while an automatic action runs; in FSC the base is zero."""
    return state.mode == "manual" and not state.stopped and state.action == "none"
