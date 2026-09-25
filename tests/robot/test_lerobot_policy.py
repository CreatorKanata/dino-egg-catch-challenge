"""tests/robot/test_lerobot_policy.py: Checks of the pick policy loader's pure parts, without torch.

A fake frozen config stands in for the checkpoint's ACTConfig, so the overrides (temporal ensembling
with n_action_steps = 1, or chunked execution clamped to chunk_size with the checkpoint's own
coefficient cleared), the warm-up timing with a fake clock, the automatic fallback to chunked
execution when ensembling is too slow for the frame, the device resolution, and the warm-up frame
are verified here. One guarded test runs the real ACTConfig in a separate interpreter (skipped
without LeRobot) to confirm the fork accepts both overrides.
"""

from dataclasses import dataclass
import os
from pathlib import Path
import subprocess
import sys
import unittest

import numpy as np

from robot.policy.lerobot_policy import _with_overrides, load_with_fallback, resolve_device, warm_up, zero_frame
from robot.policy.pick_policy import LoadedPolicy

SRC = str(Path(__file__).resolve().parents[2] / "src")
FEATURES = {"observation.state": (9,), "observation.images.wrist": (3, 4, 6)}


@dataclass(frozen=True)
class FakeACTConfig:
    chunk_size: int = 100
    n_action_steps: int = 100
    temporal_ensemble_coeff: float | None = None
    pretrained_path: str | None = None
    device: str = "cuda"


def fake_build(per_run_s):
    """A build function recording the configs it gets and reporting a fixed warm-up time."""
    configs = []

    def build(config):
        configs.append(config)
        return LoadedPolicy(infer=lambda frame: [0.0] * 9, reset=lambda: None, input_features=FEATURES,
                            n_action_steps=config.n_action_steps, device=config.device,
                            temporal_ensemble_coeff=config.temporal_ensemble_coeff, warmup_inference_s=per_run_s)
    return build, configs


class OverrideTests(unittest.TestCase):
    def test_ensembling_sets_the_coefficient_and_one_step(self):
        config = _with_overrides(FakeACTConfig(), "ckpt", "mps", 10, 0.01)
        self.assertEqual((config.temporal_ensemble_coeff, config.n_action_steps, config.device, config.pretrained_path),
                         (0.01, 1, "mps", "ckpt"))

    def test_chunked_clamps_the_horizon_and_clears_a_checkpoint_coefficient(self):
        config = _with_overrides(FakeACTConfig(temporal_ensemble_coeff=0.05, n_action_steps=1), "c", "cpu", 10, None)
        self.assertEqual((config.temporal_ensemble_coeff, config.n_action_steps), (None, 10))
        self.assertEqual(_with_overrides(FakeACTConfig(chunk_size=5), "c", "cpu", 10).n_action_steps, 5)

    def test_input_config_is_not_mutated(self):
        original = FakeACTConfig()
        _with_overrides(original, "c", "cpu", 10, 0.01)
        self.assertEqual(original, FakeACTConfig())


class WarmUpTests(unittest.TestCase):
    def test_median_of_timed_model_runs_after_the_compiling_call(self):
        ticks = iter([0.0, 0.020, 1.0, 1.030, 2.0, 2.025])  # three timed runs: 20, 30, 25 ms
        calls = []
        seconds = warm_up(lambda frame: calls.append(sorted(frame)), lambda: calls.append("reset"), FEATURES,
                          clock=lambda: next(ticks), runs=3)
        self.assertAlmostEqual(seconds, 0.025)
        self.assertEqual(calls.count("reset"), 5)  # before the first call, before each timed run, at the end
        self.assertEqual(calls[-1], "reset")  # the policy is left with a clean queue / ensembler

    def test_frame_matches_the_policy_inputs(self):
        frame = zero_frame({"observation.state": (9,), "observation.images.wrist": (3, 480, 640)})
        self.assertEqual((frame["observation.state"].shape, frame["observation.state"].dtype), ((9,), np.float32))
        self.assertEqual((frame["observation.images.wrist"].shape, frame["observation.images.wrist"].dtype),
                         ((480, 640, 3), np.uint8))


class FallbackTests(unittest.TestCase):
    def test_fast_device_keeps_ensembling(self):
        build, configs = fake_build(0.019)
        loaded = load_with_fallback(FakeACTConfig(), build, "c", "mps", 10, 0.01, max_inference_s=0.028)
        self.assertEqual((loaded.temporal_ensemble_coeff, loaded.n_action_steps, len(configs)), (0.01, 1, 1))

    def test_slow_device_falls_back_to_chunked_with_a_warning(self):
        build, configs = fake_build(0.090)
        with self.assertLogs("robot.policy.lerobot_policy", "WARNING") as logs:
            loaded = load_with_fallback(FakeACTConfig(), build, "c", "cpu", 10, 0.01, max_inference_s=0.028)
        self.assertEqual((loaded.temporal_ensemble_coeff, loaded.n_action_steps), (None, 10))
        self.assertEqual([config.temporal_ensemble_coeff for config in configs], [0.01, None])
        self.assertIn("90.0 ms", logs.output[0])

    def test_chunked_is_never_rebuilt(self):
        build, configs = fake_build(0.090)
        loaded = load_with_fallback(FakeACTConfig(), build, "c", "cpu", 10, None, max_inference_s=0.028)
        self.assertEqual((loaded.n_action_steps, len(configs)), (10, 1))


class DeviceTests(unittest.TestCase):
    def test_auto_prefers_mps(self):
        self.assertEqual(resolve_device("auto", lambda: True), "mps")
        self.assertEqual(resolve_device("auto", lambda: False), "cpu")
        self.assertEqual(resolve_device("cuda", lambda: True), "cuda")


class RealConfigTests(unittest.TestCase):
    def test_act_config_accepts_both_executions(self):
        code = ("import sys\n"
                "try:\n    from lerobot.policies.act.configuration_act import ACTConfig\n"
                "except ImportError:\n    sys.exit(77)\n"
                "from robot.policy.lerobot_policy import _with_overrides\n"
                "config = _with_overrides(ACTConfig(), 'ckpt', 'cpu', 10)\n"
                "assert (config.n_action_steps, config.chunk_size, config.device) == (10, 100, 'cpu'), config\n"
                "assert _with_overrides(ACTConfig(chunk_size=5, n_action_steps=5), 'c', 'cpu', 10).n_action_steps == 5\n"
                "ensembled = _with_overrides(ACTConfig(), 'ckpt', 'cpu', 10, 0.01)\n"
                "assert (ensembled.temporal_ensemble_coeff, ensembled.n_action_steps) == (0.01, 1), ensembled\n")
        env = {**os.environ, "PYTHONPATH": SRC}
        result = subprocess.run([sys.executable, "-c", code], env=env, capture_output=True, timeout=300)
        if result.returncode == 77:
            self.skipTest("LeRobot is not installed")
        self.assertEqual(result.returncode, 0, result.stderr.decode())


if __name__ == "__main__":
    unittest.main()
