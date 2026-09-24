"""src/robot/mode_manager.py: Operating-mode state and KachiButton command rules.

Pure and stdlib-only. Implements the decisions in docs/spec/operating-modes.md, section 2:
`Stop` is a full stop and reset in every mode and latches a stopped state that only `Go Go!`
releases (back to Manual Mode); otherwise `Go Go!` toggles Manual Mode and FSC with zero base
velocities first, and presses are ignored while an automatic action runs. `Hi!` in Manual Mode
checks the egg size and the recorded catch and release poses (start_auto_catch) and starts Auto
Catch (auto_catch.py: alignment, catch pose, wrist check, the pick-policy stub, release pose);
`Thx` checks the basket size and the recorded home pose and release motion (start_auto_release)
and starts Auto Release (auto_release.py). The pick policy and FSC are still display-only stubs.
"""

from dataclasses import dataclass, replace
from typing import Final, Literal

from robot.auto_catch import CatchOutcome, CatchRequest, CatchState, start_catch
from robot.auto_release import ReleaseOutcome, ReleaseRequest, ReleaseState, start_release
from robot.config import NOTICE_SECONDS

Mode = Literal["manual", "fsc"]
Action = Literal["none", "auto_catch", "auto_release"]
MODES: Final = ("manual", "fsc")
ACTIONS: Final = ("none", "auto_catch", "auto_release")

NOTICE_STOP: Final = "STOP"
NOTICE_MODE: Final = {"manual": "MANUAL", "fsc": "FSC"}
NOTICE_BASKET_SIZE: Final = {"none": "Basket not in view", "too_small": "Basket too far"}
NOTICE_NO_HOME: Final = "Home pose not recorded"
NOTICE_NO_MOTION: Final = "Release motion not recorded"
NOTICE_RELEASE_ALIGNING: Final = "Aligning to basket..."
NOTICE_RELEASE: Final = {
    "play_started": ("Releasing...", "info"),
    "done": ("Released!", "info"),
    "lost": ("Basket lost", "warning"),
    "align_timeout": ("Could not align", "warning"),
    "home_timeout": ("Arm did not reach home", "warning"),
    "start_timeout": ("Arm did not reach the release start", "warning"),
}
NOTICE_LISTENING: Final = "Listening..."
NOTICE_VOICE_ENDED: Final = "Voice input ended"
NOTICE_SIZE: Final = {"none": "No egg in view", "too_small": "Egg too far", "too_large": "Egg too close"}
NOTICE_ALIGNING: Final = "Aligning..."
NOTICE_NO_CATCH: Final = "Catch pose not recorded"
NOTICE_CATCH: Final = {
    "lost": ("Egg lost", "warning"),
    "align_timeout": ("Could not align", "warning"),
    "catch_timeout": ("Arm did not reach the catch pose", "warning"),
    "no_wrist_egg": ("Egg not in wrist view", "warning"),
    "policy_stub": ("Catch: policy not available yet", "info"),
    "release_timeout": ("Arm did not reach the release pose", "warning"),
    "done": ("Ready", "info"),  # only after a successful pass (the stub counts as success)
    "catch_timeout_returned": ("Arm did not reach the catch pose", "warning"),  # shown again once back
    "no_wrist_egg_returned": ("Egg not in wrist view", "warning"),
}
CATCH_TERMINAL: Final = ("lost", "align_timeout", "release_timeout", "done", "catch_timeout_returned",
                         "no_wrist_egg_returned")
NOTICE_CAPTURED: Final = "Captured"
NOTICE_CAPTURE_FAILED: Final = "Capture failed"

NoticeLevel = Literal["info", "warning"]
NOTICE_LEVELS: Final = ("info", "warning")


@dataclass(frozen=True)
class AppState:
    """Current mode, running action and its state (Auto Catch, Auto Release), FSC voice input,
    notice, and the Stop flag."""

    mode: Mode = "manual"
    action: Action = "none"
    voice_listening: bool = False
    notice: str = ""
    notice_until: float = 0.0
    stopped: bool = False
    notice_level: NoticeLevel = "info"
    catch: CatchState = CatchState()
    release: ReleaseState = ReleaseState()


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
    if state.mode == "manual":  # needs the egg size and poses: the loop routes it to start_auto_catch
        return Transition(state=state)
    listening = not state.voice_listening
    notice = NOTICE_LISTENING if listening else NOTICE_VOICE_ENDED
    return Transition(state=with_notice(replace(state, voice_listening=listening), notice, now, notice_s))


def _thx(state: AppState, now: float, notice_s: float) -> Transition:
    # Manual Mode needs the basket and the data files: the loop routes it to start_auto_release.
    return Transition(state=state)  # owner decision: no action in FSC


_HANDLERS: Final = {"mode_toggle": _toggle_mode, "hi": _hi, "thx": _thx}


def apply_command(state: AppState, command: str, now: float, notice_s: float = NOTICE_SECONDS) -> Transition:
    """Apply one KachiButton command at time `now`; never mutates `state`.

    `stop` always works, latches `stopped`, and cancels any action. While stopped, only
    `mode_toggle` does anything: it resumes Manual Mode. Other known commands are ignored while
    an action runs. Unknown commands leave the state unchanged. `hi` and `thx` in Manual Mode are
    left unchanged here because they need the egg size and the recorded poses (start_auto_catch) or
    the basket and the recorded data (start_auto_release).
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


def start_auto_catch(
    state: AppState, request: CatchRequest, now: float, notice_s: float = NOTICE_SECONDS
) -> Transition:
    """`Hi!` in Manual Mode with the latest front-camera size class and the recorded poses.

    A size other than "ok", a missing catch pose, or a missing release pose (home) shows a warning
    notice and nothing moves; otherwise the alignment starts. Only in Manual Mode, not stopped, and
    with no action running; otherwise the state is unchanged.
    """
    if state.mode != "manual" or state.stopped or state.action != "none":
        return Transition(state=state)
    if request.size != "ok":
        return Transition(state=with_notice(state, NOTICE_SIZE[request.size], now, notice_s, "warning"))
    if request.catch is None:
        return Transition(state=with_notice(state, NOTICE_NO_CATCH, now, notice_s, "warning"))
    if request.home is None:
        return Transition(state=with_notice(state, NOTICE_NO_HOME, now, notice_s, "warning"))
    started = replace(state, action="auto_catch", catch=start_catch(now, request.catch, request.home))
    return Transition(state=with_notice(started, NOTICE_ALIGNING, now, notice_s), disengage_arm=True)


def catch_notice(state: AppState, outcome: CatchOutcome, now: float, notice_s: float = NOTICE_SECONDS) -> Transition:
    """Apply an Auto Catch outcome: "running" changes nothing, a non-terminal outcome only shows its
    notice (the action goes on), and a terminal outcome returns to Manual Mode with zero base
    velocities this frame and arm following disengaged (it re-syncs to the leader slowly)."""
    if outcome not in NOTICE_CATCH or state.action != "auto_catch":
        return Transition(state=state)
    notice, level = NOTICE_CATCH[outcome]
    if outcome not in CATCH_TERMINAL:
        return Transition(state=with_notice(state, notice, now, notice_s, level))
    ended = replace(state, action="none", catch=CatchState())
    return Transition(state=with_notice(ended, notice, now, notice_s, level), stop_base=True, disengage_arm=True)


def start_auto_release(
    state: AppState, request: ReleaseRequest, now: float, notice_s: float = NOTICE_SECONDS
) -> Transition:
    """`Thx` in Manual Mode with the latest basket size class and the loaded home pose and motion.

    No usable basket or a missing file shows a warning notice and nothing moves; otherwise the
    basket alignment starts. Only in Manual Mode, not stopped, and with no action running.
    """
    if state.mode != "manual" or state.stopped or state.action != "none":
        return Transition(state=state)
    if request.size != "ok":
        return Transition(state=with_notice(state, NOTICE_BASKET_SIZE[request.size], now, notice_s, "warning"))
    if request.home is None:
        return Transition(state=with_notice(state, NOTICE_NO_HOME, now, notice_s, "warning"))
    if not request.frames:
        return Transition(state=with_notice(state, NOTICE_NO_MOTION, now, notice_s, "warning"))
    started = replace(state, action="auto_release", release=start_release(now, request.home, request.frames))
    return Transition(state=with_notice(started, NOTICE_RELEASE_ALIGNING, now, notice_s), disengage_arm=True)


def release_notice(state: AppState, outcome: ReleaseOutcome, now: float, notice_s: float = NOTICE_SECONDS) -> Transition:
    """Apply a release outcome: "running" changes nothing, "play_started" only shows "Releasing...",
    and a terminal outcome returns to Manual Mode with zero base velocities this frame and arm
    following disengaged (it re-syncs to the leader slowly)."""
    if outcome not in NOTICE_RELEASE or state.action != "auto_release":
        return Transition(state=state)
    notice, level = NOTICE_RELEASE[outcome]
    if outcome == "play_started":
        return Transition(state=with_notice(state, notice, now, notice_s, level))
    ended = replace(state, action="none", release=ReleaseState())
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
