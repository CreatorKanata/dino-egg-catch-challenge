"""src/robot/policy/pick_policy.py: The pick policy runner Auto Catch calls in its `pick` phase.

Loads a trained checkpoint (CreatorKanata/act_dino_pick_egg on the Hub, or a local directory)
through a loader (lerobot_policy.py by default; a fake in tests), refuses policies that read
anything but the wrist and front images and the state, and turns one LeKiwi observation into one
action per loop frame (ACT runs the model once every n_action_steps frames and returns the queued
actions in between). Timing: the first inference and then the running average are logged at INFO,
at most once per PICK_TIMING_LOG_INTERVAL_S.

RGB ORDER (the most likely silent bug): the policy must receive the raw frames exactly as
LeKiwiClient returns them, RGB-ordered, because the dataset stored those arrays. The loop's
BGR copies (vision/frames.py; for the detector, signboard, captures, and Rerun) must never reach
the policy. The state is the nine-value vector in LeKiwiClient._state_order (six arm keys, then
x.vel, y.vel, theta.vel), as recorded. torch and LeRobot are never imported here (numpy only).
"""

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
import logging
import time
from typing import Any

import numpy as np

from robot.config import ARM_KEYS
from robot.controller_to_action import BASE_KEYS
from robot.policy.config_policy import (
    PICK_ACTION_HORIZON,
    PICK_POLICY_DEVICE,
    PICK_POLICY_FEATURES,
    PICK_TIMING_LOG_INTERVAL_S,
)

logger = logging.getLogger(__name__)

STATE_FEATURE = "observation.state"
IMAGE_PREFIX = "observation.images."
STATE_ORDER = (*ARM_KEYS, *BASE_KEYS)  # LeKiwiClient._state_order: the recorded state and action order
ACTION_KEYS = STATE_ORDER


@dataclass(frozen=True)
class LoadedPolicy:
    """A checkpoint ready for inference: `infer(frame) -> action values` (policy order), `reset()`,
    the input features with their shapes (images C, H, W), actions per inference, the device, the
    policy type, and the output (action) feature shapes."""

    infer: Callable[[Mapping[str, Any]], Sequence[float]]
    reset: Callable[[], None]
    input_features: Mapping[str, tuple[int, ...]]
    n_action_steps: int
    device: str
    policy_type: str = ""
    action_shapes: tuple[tuple[int, ...], ...] = ()


@dataclass(frozen=True)
class InferenceTiming:
    """Model runs so far, their total and first duration in seconds, and the last log time."""

    count: int = 0
    total_s: float = 0.0
    first_s: float = 0.0
    logged_at: float | None = None


Loader = Callable[[str, str, int], LoadedPolicy]


def _lerobot_loader(path: str, device: str, horizon: int) -> LoadedPolicy:
    from robot.policy.lerobot_policy import load_lerobot_policy  # torch and LeRobot load only here

    return load_lerobot_policy(path, device, horizon)


def _image(observation: Mapping[str, Any], feature: str, shape: tuple[int, ...]) -> Any:
    camera = feature.removeprefix(IMAGE_PREFIX)
    if observation.get(camera) is None:
        raise KeyError(f"Observation lacks the {camera!r} frame the pick policy needs")
    frame = observation[camera]  # the raw RGB array, passed on untouched
    if len(shape) == 3 and tuple(np.shape(frame)) != (shape[1], shape[2], shape[0]):
        raise ValueError(f"{camera} frame is {np.shape(frame)}, the policy expects {shape[1]}x{shape[2]}")
    return frame


def build_frame(observation: Mapping[str, Any], features: Mapping[str, tuple[int, ...]]) -> dict[str, Any]:
    """The policy's input from a LeKiwi observation: the state vector (float32, STATE_ORDER) and
    each image the policy reads as the client's HWC uint8 RGB array itself (never a BGR copy)."""
    frame: dict[str, Any] = {}
    for feature, shape in features.items():
        if feature == STATE_FEATURE:
            missing = [key for key in STATE_ORDER if key not in observation]
            if missing:
                raise KeyError(f"Observation lacks state keys: {', '.join(missing)}")
            frame[feature] = np.array([float(observation[key]) for key in STATE_ORDER], dtype=np.float32)
        else:
            frame[feature] = _image(observation, feature, shape)
    return frame


def action_dict(values: Sequence[float]) -> dict[str, float]:
    """The nine action keys from the policy's output vector (STATE_ORDER)."""
    if len(values) != len(ACTION_KEYS):
        raise ValueError(f"Policy returned {len(values)} action values, expected {len(ACTION_KEYS)}")
    return {key: float(value) for key, value in zip(ACTION_KEYS, values)}


def check_policy(loaded: LoadedPolicy) -> None:
    """Refuse inputs the runner cannot build and actions that are not the nine LeKiwi keys."""
    unknown = sorted(set(loaded.input_features) - PICK_POLICY_FEATURES)
    if unknown:
        raise ValueError(f"Pick policy reads unsupported features: {', '.join(unknown)}")
    if loaded.action_shapes and loaded.action_shapes != ((len(ACTION_KEYS),),):
        raise ValueError(f"Pick policy action shape {loaded.action_shapes} is not the nine LeKiwi keys")


def record_inference(timing: InferenceTiming, seconds: float, now: float,
                     interval_s: float = PICK_TIMING_LOG_INTERVAL_S) -> tuple[InferenceTiming, str]:
    """The timing after one model run and the message to log now ("" = nothing yet)."""
    count, total = timing.count + 1, timing.total_s + seconds
    first = seconds if timing.count == 0 else timing.first_s
    if timing.logged_at is not None and now - timing.logged_at < interval_s:
        return InferenceTiming(count, total, first, timing.logged_at), ""
    message = (f"Pick policy: first inference {1000 * seconds:.1f} ms" if count == 1 else
               f"Pick policy: average inference {1000 * total / count:.1f} ms over {count} runs")
    return InferenceTiming(count, total, first, now), message


class PickPolicy:
    """Load once at startup; reset() at the start of every pick; act() once per loop frame."""

    def __init__(self, path: str, device: str = PICK_POLICY_DEVICE, horizon: int = PICK_ACTION_HORIZON,
                 loader: Loader = _lerobot_loader, clock: Callable[[], float] = time.perf_counter) -> None:
        self.path, self._device, self._horizon, self._loader, self._clock = path, device, horizon, loader, clock
        self._loaded: LoadedPolicy | None = None
        self._frames = 0  # act() calls since the last reset
        self.timing = InferenceTiming()

    def load(self) -> LoadedPolicy:
        """Load and check the checkpoint; log what was loaded and how long it took. Errors propagate."""
        started = self._clock()
        loaded = self._loader(self.path, self._device, self._horizon)
        check_policy(loaded)
        self._loaded = loaded
        logger.info("Pick policy %s (%s) on %s: inputs %s, n_action_steps %d, loaded in %.1f s", self.path,
                    loaded.policy_type or "?", loaded.device, sorted(loaded.input_features), loaded.n_action_steps,
                    self._clock() - started)
        return loaded

    def _require(self) -> LoadedPolicy:
        if self._loaded is None:
            raise RuntimeError("Pick policy is not loaded; call load() first")
        return self._loaded

    def reset(self) -> None:
        """Clear the action queue (a new pick starts from a fresh observation)."""
        self._require().reset()
        self._frames = 0

    def act(self, observation: Mapping[str, Any]) -> dict[str, float]:
        """The nine action keys for this frame from the raw (RGB) LeKiwi observation."""
        loaded = self._require()
        frame = build_frame(observation, loaded.input_features)
        started = self._clock()
        values = loaded.infer(frame)
        finished = self._clock()
        if self._frames % max(1, loaded.n_action_steps) == 0:  # this call ran the model
            self.timing, message = record_inference(self.timing, finished - started, finished)
            if message:
                logger.info(message)
        self._frames += 1
        return action_dict(values)
