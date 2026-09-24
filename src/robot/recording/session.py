"""src/robot/recording/session.py: Pure planning and bookkeeping of a pick_egg recording session.

SessionPlan is built once from the CLI arguments; SessionState counts recorded, skipped, and
re-recorded episodes and the index of the next attempt (a skipped attempt still uses up an index,
so a session ends after num_episodes attempts or a stop). The dataset root and start checks and
the summary text are here too, so record_pick_egg.py stays thin. Stdlib-only; nothing mutates.
"""

from collections.abc import Sequence
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

from robot.recording.config_recording import EGG_COLORS


@dataclass(frozen=True)
class SessionPlan:
    """What to record: dataset, episode count and timing, base task, egg color, and upload."""

    repo_id: str
    num_episodes: int
    episode_time_s: float
    reset_time_s: float
    fps: int
    task: str
    color: str
    push: bool

    @property
    def single_task(self) -> str:
        """The task string stored with every frame (includes the egg color for filtering)."""
        return task_with_color(self.task, self.color)


@dataclass(frozen=True)
class SessionState:
    """Progress so far; `index` is the zero-based next attempt, `durations_s` one entry per saved episode."""

    recorded: int = 0
    skipped: int = 0
    index: int = 0
    rerecorded: int = 0
    stopped: bool = False
    durations_s: tuple[float, ...] = ()


def task_with_color(task: str, color: str) -> str:
    """`Pick up the egg with the mouth` + `red` -> `Pick up the egg with the mouth (red egg)`."""
    return f"{task.strip()} ({color} egg)"


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def plan_from_args(args: Any) -> SessionPlan:
    """Build and validate the plan from parsed CLI arguments (argparse.Namespace). Raises ValueError."""
    plan = SessionPlan(
        repo_id=str(args.repo_id),
        num_episodes=int(args.num_episodes),
        episode_time_s=float(args.episode_time_s),
        reset_time_s=float(args.reset_time_s),
        fps=int(args.fps),
        task=str(args.task),
        color=str(args.egg_color),
        push=not bool(args.no_push),
    )
    _require(plan.repo_id.count("/") == 1 and all(plan.repo_id.split("/")), "--repo-id must be <owner>/<name>")
    _require(plan.num_episodes >= 1, "--num-episodes must be at least 1")
    _require(plan.episode_time_s > 0, "--episode-time-s must be positive")
    _require(plan.reset_time_s >= 0, "--reset-time-s must not be negative")
    _require(plan.fps >= 1, "--fps must be at least 1")
    _require(bool(plan.task.strip()), "--task must not be empty")
    _require(plan.color in EGG_COLORS, f"--egg-color must be one of {', '.join(EGG_COLORS)}")
    return plan


def after_recorded(state: SessionState, duration_s: float) -> SessionState:
    """An episode was saved: count it, keep its duration, move to the next attempt."""
    return replace(state, recorded=state.recorded + 1, index=state.index + 1,
                   durations_s=(*state.durations_s, float(duration_s)))


def after_skipped(state: SessionState) -> SessionState:
    """The start-pose gate timed out: nothing recorded, move to the next attempt."""
    return replace(state, skipped=state.skipped + 1, index=state.index + 1)


def after_rerecord(state: SessionState) -> SessionState:
    """The operator discarded the episode: record the same attempt again."""
    return replace(state, rerecorded=state.rerecorded + 1)


def after_stop(state: SessionState) -> SessionState:
    """Esc, a stop during the gate, or Ctrl+C: the session ends."""
    return replace(state, stopped=True)


def is_finished(plan: SessionPlan, state: SessionState) -> bool:
    return state.stopped or state.index >= plan.num_episodes


def is_last_episode(plan: SessionPlan, state: SessionState) -> bool:
    """True for the final attempt, after which no reset phase runs (as in the fork's CLI)."""
    return state.index >= plan.num_episodes - 1


def episode_number(state: SessionState) -> int:
    """One-based number of the current attempt, for announcements."""
    return state.index + 1


def recording_message(plan: SessionPlan, state: SessionState) -> str:
    return f"Recording episode {episode_number(state)} of {plan.num_episodes}, {plan.color} egg"


def skip_message(state: SessionState) -> str:
    return f"Leader not at the catch pose in time, skipping episode {episode_number(state)}"


def resolve_root(repo_id: str, root: str | None, lerobot_home: Path) -> Path:
    """The dataset directory: `--root` when given, else LeRobot's default `<HF_LEROBOT_HOME>/<repo_id>`."""
    return Path(root) if root else lerobot_home / repo_id


def start_error(resume: bool, root_exists: bool, root: Path) -> str | None:
    """Why the session cannot start with this dataset directory, or None."""
    if resume and not root_exists:
        return f"--resume given but there is no dataset at {root}"
    if not resume and root_exists:
        return f"A dataset already exists at {root}; add --resume to append episodes or pass another --root"
    return None


def _durations_line(durations: Sequence[float]) -> str:
    if not durations:
        return "Episode durations: none"
    listed = ", ".join(f"{value:.1f} s" for value in durations)
    return f"Episode durations: {listed} (mean {sum(durations) / len(durations):.1f} s)"


def summary_lines(plan: SessionPlan, state: SessionState, root: Path, pushed: bool) -> tuple[str, ...]:
    """The session summary printed at exit."""
    ending = "stopped early" if state.stopped and state.index < plan.num_episodes else "completed"
    return (
        f"Session {ending}: task '{plan.single_task}'",
        f"Recorded: {state.recorded} of {plan.num_episodes}",
        f"Skipped: {state.skipped}",
        f"Re-recorded: {state.rerecorded}",
        _durations_line(state.durations_s),
        f"Dataset root: {root}",
        f"Pushed to the Hub: {'yes (' + plan.repo_id + ')' if pushed else 'no'}",
    )
