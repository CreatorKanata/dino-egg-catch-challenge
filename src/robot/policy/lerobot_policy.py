"""src/robot/policy/lerobot_policy.py: Load a LeRobot policy checkpoint for the pick runner.

Mirrors the fork's rollout path (rollout/context.py build_rollout_context and
rollout/inference/sync.py SyncInferenceEngine, verified 2026-09-25): the policy config is read from
the checkpoint (Hub id or local directory), the device is set on it, the weights are loaded with
the policy class's from_pretrained, and the pre/post processors come from
make_pre_post_processors(pretrained_path=...) with only the device overridden, so the
normalization stats bundled in the checkpoint are used (no dataset is needed). One call =
prepare_observation_for_inference (HWC uint8 -> CHW float in [0, 1] with a batch dimension) ->
preprocessor -> select_action -> postprocessor, under inference_mode (autocast only on CUDA with
use_amp, as the fork).
Execution: with a temporal ensembling coefficient the ACT config gets temporal_ensemble_coeff and
n_action_steps = 1 (ACTConfig.__post_init__ refuses anything else; select_action then predicts a
chunk every call and ACTTemporalEnsembler averages it, and policy.reset() clears the ensembler);
without one, n_action_steps = min(horizon, chunk_size). Warm-up at load: one inference on a zero
frame (the first MPS call of a fresh process took up to about 5 s on this Mac, 2026-09-25, which in
the pick phase would block the 30 Hz loop and Stop), then WARMUP_TIMED_RUNS timed model runs; if
ensembling needs more than PICK_MAX_INFERENCE_S per frame the policy is rebuilt chunked with a
WARNING. torch and LeRobot are imported inside load_lerobot_policy only, so importing this module
stays light; the tests exercise the pure parts with fakes.
"""

from collections.abc import Callable, Mapping, Sequence
from contextlib import nullcontext
import dataclasses
import logging
import statistics
import time
from typing import Any, Final

import numpy as np

from robot.policy.config_policy import PICK_MAX_INFERENCE_S
from robot.policy.pick_policy import STATE_FEATURE, LoadedPolicy

logger = logging.getLogger(__name__)

WARMUP_TIMED_RUNS: Final = 3  # model runs timed after the first (compiling) warm-up call; the median counts
Build = Callable[[Any], LoadedPolicy]  # a config with overrides -> a warmed-up policy with its measured time


def resolve_device(requested: str, mps_available: Callable[[], bool]) -> str:
    """"auto" -> "mps" when available, else "cpu"; anything else is returned unchanged."""
    if requested != "auto":
        return requested
    return "mps" if mps_available() else "cpu"


def _with_overrides(config: Any, path: str, device: str, horizon: int, ensemble_coeff: float | None = None) -> Any:
    """A copy of the checkpoint config with the path and device and, for ACT (a config with
    n_action_steps), either temporal ensembling (coeff, n_action_steps = 1) or chunked execution
    (no ensembling, n_action_steps = min(horizon, chunk_size)); the coefficient given here wins over
    the checkpoint's."""
    changes: dict[str, Any] = {"pretrained_path": path, "device": device}
    if hasattr(config, "n_action_steps"):
        chunk = int(getattr(config, "chunk_size", horizon))
        if ensemble_coeff is not None and hasattr(config, "temporal_ensemble_coeff"):
            changes.update(temporal_ensemble_coeff=float(ensemble_coeff), n_action_steps=1)
        else:
            changes.update(n_action_steps=max(1, min(horizon, chunk)))
            if hasattr(config, "temporal_ensemble_coeff"):
                changes["temporal_ensemble_coeff"] = None
    return dataclasses.replace(config, **changes)


def zero_frame(features: Mapping[str, tuple[int, ...]]) -> dict[str, Any]:
    """A policy input of zeros: float32 state, HWC uint8 images (from their C, H, W shapes)."""
    return {name: np.zeros(shape, dtype=np.float32) if name == STATE_FEATURE or len(shape) != 3
            else np.zeros((shape[1], shape[2], shape[0]), dtype=np.uint8) for name, shape in features.items()}


def warm_up(infer: Callable[[Mapping[str, Any]], Sequence[float]], reset: Callable[[], None],
            features: Mapping[str, tuple[int, ...]], clock: Callable[[], float] = time.perf_counter,
            runs: int = WARMUP_TIMED_RUNS) -> float:
    """Run the compiling first call, then `runs` timed model runs (a reset before each, so each one
    runs the model); leave the policy reset. Returns the median seconds per model run."""
    frame = zero_frame(features)
    reset()
    infer(frame)
    durations = []
    for _ in range(max(1, runs)):
        reset()
        started = clock()
        infer(frame)
        durations.append(clock() - started)
    reset()
    return statistics.median(durations)


def load_with_fallback(checkpoint: Any, build: Build, path: str, device: str, horizon: int,
                       ensemble_coeff: float | None, max_inference_s: float = PICK_MAX_INFERENCE_S) -> LoadedPolicy:
    """Build with the requested execution; when ensembling (a model run every frame) is slower than
    max_inference_s at warm-up, warn and rebuild chunked so a slow device still works."""
    loaded = build(_with_overrides(checkpoint, path, device, horizon, ensemble_coeff))
    if loaded.temporal_ensemble_coeff is None or loaded.warmup_inference_s <= max_inference_s:
        return loaded
    logger.warning("Pick policy inference takes %.1f ms on %s, over the %.0f ms per-frame limit for temporal "
                   "ensembling; falling back to chunked execution", 1000 * loaded.warmup_inference_s, device,
                   1000 * max_inference_s)
    return build(_with_overrides(checkpoint, path, device, horizon, None))


def load_lerobot_policy(path: str, device: str, horizon: int, ensemble_coeff: float | None = None) -> LoadedPolicy:
    """The checkpoint at `path` ready for inference on `device` ("auto" resolved here)."""
    import torch
    import lerobot.policies  # noqa: F401  registers the policy config types ('act', ...) that
    # PreTrainedConfig.from_pretrained resolves; without it the lazy load fails with "not registered".
    from lerobot.configs.policies import PreTrainedConfig

    resolved = resolve_device(device, torch.backends.mps.is_available)
    return load_with_fallback(PreTrainedConfig.from_pretrained(path), lambda config: _build(path, config, resolved),
                              path, resolved, horizon, ensemble_coeff)


def _build(path: str, config: Any, device: str) -> LoadedPolicy:
    import torch
    from lerobot.policies import get_policy_class, make_pre_post_processors
    from lerobot.policies.utils import prepare_observation_for_inference

    policy = get_policy_class(config.type).from_pretrained(path, config=config)
    policy = policy.to(device)
    policy.eval()
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=config, pretrained_path=path, preprocessor_overrides={"device_processor": {"device": device}})
    torch_device = torch.device(device)
    use_amp = torch_device.type == "cuda" and bool(getattr(config, "use_amp", False))

    def infer(frame: Mapping[str, Any]) -> Sequence[float]:
        observation = dict(frame)  # shallow copy: the caller's arrays are read, never modified
        with torch.inference_mode(), torch.autocast(device_type="cuda") if use_amp else nullcontext():
            batch = preprocessor(prepare_observation_for_inference(observation, torch_device))
            action = postprocessor(policy.select_action(batch))
        return action.squeeze(0).cpu().tolist()

    def reset() -> None:
        policy.reset()  # clears the ACT action queue or its temporal ensembler
        preprocessor.reset()
        postprocessor.reset()

    features = {name: tuple(int(size) for size in feature.shape) for name, feature in config.input_features.items()}
    per_run_s = warm_up(infer, reset, features)
    return LoadedPolicy(infer=infer, reset=reset, input_features=features,
                        n_action_steps=int(getattr(config, "n_action_steps", 1)), device=device,
                        policy_type=str(config.type),
                        action_shapes=tuple(tuple(feature.shape) for feature in config.output_features.values()),
                        temporal_ensemble_coeff=getattr(config, "temporal_ensemble_coeff", None),
                        warmup_inference_s=per_run_s)
