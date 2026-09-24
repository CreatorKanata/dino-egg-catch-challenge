"""src/robot/policy/lerobot_policy.py: Load a LeRobot policy checkpoint for the pick runner.

Mirrors the fork's rollout path (rollout/context.py build_rollout_context and
rollout/inference/sync.py SyncInferenceEngine, verified 2026-09-25): the policy config is read from
the checkpoint (Hub id or local directory), the device is set on it, the weights are loaded with
the policy class's from_pretrained, and the pre/post processors come from
make_pre_post_processors(pretrained_path=...) with only the device overridden, so the
normalization stats bundled in the checkpoint are used (no dataset is needed). One call =
prepare_observation_for_inference (HWC uint8 -> CHW float in [0, 1] with a batch dimension) ->
preprocessor -> select_action -> postprocessor, under inference_mode (autocast only on CUDA with
use_amp, as the fork). One warm-up inference on a zero frame runs at load, then the policy is
reset: on this Mac the first MPS call of a fresh process took up to about 5 s (2026-09-25), which
in the pick phase would block the 30 Hz loop, and with it Stop. torch and LeRobot are imported
inside the function only, so importing this module stays light; the tests never call it.
"""

from collections.abc import Callable, Mapping, Sequence
from contextlib import nullcontext
import dataclasses
import logging
import time
from typing import Any

import numpy as np

from robot.policy.pick_policy import STATE_FEATURE, LoadedPolicy

logger = logging.getLogger(__name__)


def resolve_device(requested: str, mps_available: Callable[[], bool]) -> str:
    """"auto" -> "mps" when available, else "cpu"; anything else is returned unchanged."""
    if requested != "auto":
        return requested
    return "mps" if mps_available() else "cpu"


def _with_overrides(config: Any, path: str, device: str, horizon: int) -> Any:
    """A copy of the checkpoint config with the path, device, and (ACT without temporal ensembling)
    n_action_steps = min(horizon, chunk_size)."""
    changes: dict[str, Any] = {"pretrained_path": path, "device": device}
    if hasattr(config, "n_action_steps") and getattr(config, "temporal_ensemble_coeff", None) is None:
        changes["n_action_steps"] = max(1, min(horizon, int(getattr(config, "chunk_size", horizon))))
    return dataclasses.replace(config, **changes)


def zero_frame(features: Mapping[str, tuple[int, ...]]) -> dict[str, Any]:
    """A policy input of zeros: float32 state, HWC uint8 images (from their C, H, W shapes)."""
    return {name: np.zeros(shape, dtype=np.float32) if name == STATE_FEATURE or len(shape) != 3
            else np.zeros((shape[1], shape[2], shape[0]), dtype=np.uint8) for name, shape in features.items()}


def load_lerobot_policy(path: str, device: str, horizon: int) -> LoadedPolicy:
    """The checkpoint at `path` ready for inference on `device` ("auto" resolved here)."""
    import torch
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies import get_policy_class, make_pre_post_processors
    from lerobot.policies.utils import prepare_observation_for_inference

    resolved = resolve_device(device, torch.backends.mps.is_available)
    config = _with_overrides(PreTrainedConfig.from_pretrained(path), path, resolved, horizon)
    policy = get_policy_class(config.type).from_pretrained(path, config=config)
    policy = policy.to(resolved)
    policy.eval()
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=config, pretrained_path=path, preprocessor_overrides={"device_processor": {"device": resolved}})
    torch_device = torch.device(resolved)
    use_amp = torch_device.type == "cuda" and bool(getattr(config, "use_amp", False))

    def infer(frame: Mapping[str, Any]) -> Sequence[float]:
        observation = dict(frame)  # shallow copy: the caller's arrays are read, never modified
        with torch.inference_mode(), torch.autocast(device_type="cuda") if use_amp else nullcontext():
            batch = preprocessor(prepare_observation_for_inference(observation, torch_device))
            action = postprocessor(policy.select_action(batch))
        return action.squeeze(0).cpu().tolist()

    def reset() -> None:
        policy.reset()
        preprocessor.reset()
        postprocessor.reset()

    features = {name: tuple(int(size) for size in feature.shape) for name, feature in config.input_features.items()}
    started = time.perf_counter()
    infer(zero_frame(features))  # warm-up (MPS kernel compilation), then a clean queue
    reset()
    logger.info("Pick policy warm-up inference took %.1f s on %s", time.perf_counter() - started, resolved)
    action_shapes = [tuple(feature.shape) for feature in config.output_features.values()]
    steps = int(getattr(config, "n_action_steps", 1))
    return LoadedPolicy(infer=infer, reset=reset, input_features=features, n_action_steps=steps, device=resolved,
                        policy_type=str(config.type), action_shapes=tuple(action_shapes))
