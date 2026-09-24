"""tests/robot/test_pick_policy.py: Checks of the pick policy runner without torch or a checkpoint.

A fake loader stands in for the LeRobot checkpoint, so the observation assembly (state vector in
LeKiwiClient's `_state_order`, the raw RGB frame passed through untouched, only the features the
policy reads), the refusal of unknown features and mismatched frames, the nine-key action mapping,
reset, the timing log, the device resolution, and the warm-up frame are verified in this process. One guarded test
runs the real ACT config in a separate interpreter (skipped without LeRobot) to confirm the
`n_action_steps` override the loader applies.
"""

import os
from pathlib import Path
import subprocess
import sys
import unittest

import numpy as np

from robot.policy.lerobot_policy import resolve_device, zero_frame
from robot.policy.pick_policy import ACTION_KEYS, LoadedPolicy, PickPolicy, build_frame

SRC = str(Path(__file__).resolve().parents[2] / "src")
# LeKiwiClient._state_order in the fork (robots/lekiwi/lekiwi_client.py), copied, not imported.
FORK_STATE_ORDER = ("arm_shoulder_pan.pos", "arm_shoulder_lift.pos", "arm_elbow_flex.pos", "arm_wrist_flex.pos",
                    "arm_wrist_roll.pos", "arm_gripper.pos", "x.vel", "y.vel", "theta.vel")
WRIST_ONLY = {"observation.state": (9,), "observation.images.wrist": (3, 2, 2)}
WRIST_RGB = np.array([[[255, 0, 0], [0, 255, 0]], [[0, 0, 255], [9, 9, 9]]], dtype=np.uint8)  # red first


def observation():
    """A LeKiwiClient-like observation: nine state keys (values = their index), frames, and extras."""
    return {**{key: float(index) for index, key in enumerate(FORK_STATE_ORDER)}, "observation.state": "ignored",
            "wrist": WRIST_RGB, "front": np.zeros((2, 2, 3), dtype=np.uint8)}


class FakeLoader:
    """Records the frames it is given and returns a fixed nine-value action per call."""

    def __init__(self, features=WRIST_ONLY, output=tuple(range(9)), steps=10, fail=None, action=(9,)):
        self.features, self.output, self.steps, self.fail, self.action = features, output, steps, fail, action
        self.frames, self.resets, self.calls = [], 0, []

    def __call__(self, path, device, horizon):
        self.calls.append((path, device, horizon))
        if self.fail is not None:
            raise self.fail
        return LoadedPolicy(infer=self.infer, reset=self.reset, input_features=self.features, n_action_steps=self.steps,
                            device="cpu", policy_type="act", action_shapes=(self.action,))

    def infer(self, frame):
        self.frames.append(frame)
        return list(self.output)

    def reset(self):
        self.resets += 1


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        self.now += 0.05  # every call to the clock advances 50 ms
        return self.now


class BuildFrameTests(unittest.TestCase):
    def test_state_vector_follows_the_client_state_order(self):
        frame = build_frame(observation(), WRIST_ONLY)
        np.testing.assert_array_equal(frame["observation.state"], np.arange(9, dtype=np.float32))
        self.assertEqual(frame["observation.state"].dtype, np.float32)

    def test_image_is_the_raw_rgb_array_untouched(self):
        raw = observation()
        frame = build_frame(raw, WRIST_ONLY)
        self.assertIs(frame["observation.images.wrist"], raw["wrist"])  # not a BGR copy
        np.testing.assert_array_equal(frame["observation.images.wrist"][0, 0], [255, 0, 0])
        self.assertEqual(set(frame), set(WRIST_ONLY))  # front is not sent to a wrist-only policy

    def test_missing_inputs_and_wrong_frame_size_raise(self):
        with self.assertRaisesRegex(KeyError, "wrist"):
            build_frame({key: value for key, value in observation().items() if key != "wrist"}, WRIST_ONLY)
        with self.assertRaisesRegex(KeyError, "x.vel"):
            build_frame({key: value for key, value in observation().items() if key != "x.vel"}, WRIST_ONLY)
        with self.assertRaisesRegex(ValueError, "480x640"):
            build_frame(observation(), {"observation.images.wrist": (3, 480, 640)})


class PickPolicyTests(unittest.TestCase):
    def test_load_passes_path_device_horizon_and_reports_the_policy(self):
        loader = FakeLoader()
        runner = PickPolicy("local/ckpt", device="auto", horizon=7, loader=loader, clock=Clock())
        with self.assertLogs("robot.policy.pick_policy", "INFO") as logs:
            loaded = runner.load()
        self.assertEqual(loader.calls, [("local/ckpt", "auto", 7)])
        self.assertEqual((loaded.device, loaded.n_action_steps), ("cpu", 10))
        self.assertIn("observation.images.wrist", "\n".join(logs.output))

    def test_load_refuses_unknown_features_and_actions(self):
        for loader, pattern in ((FakeLoader(features={"observation.images.top": (3, 2, 2)}), "observation.images.top"),
                                (FakeLoader(action=(7,)), "nine")):
            with self.subTest(pattern=pattern), self.assertRaisesRegex(ValueError, pattern):
                PickPolicy("p", loader=loader).load()

    def test_load_failure_propagates(self):
        with self.assertRaisesRegex(OSError, "offline"):
            PickPolicy("p", loader=FakeLoader(fail=OSError("offline"))).load()

    def test_act_maps_the_output_to_the_nine_action_keys(self):
        loader = FakeLoader()
        runner = PickPolicy("p", loader=loader, clock=Clock())
        runner.load()
        action = runner.act(observation())
        self.assertEqual(ACTION_KEYS, FORK_STATE_ORDER)
        self.assertEqual(action, {key: float(index) for index, key in enumerate(FORK_STATE_ORDER)})
        self.assertIs(loader.frames[0]["observation.images.wrist"], WRIST_RGB)

    def test_act_rejects_a_wrong_action_length_and_needs_load(self):
        with self.assertRaisesRegex(RuntimeError, "load"):
            PickPolicy("p", loader=FakeLoader()).act(observation())
        runner = PickPolicy("p", loader=FakeLoader(output=(1.0, 2.0)), clock=Clock())
        runner.load()
        with self.assertRaisesRegex(ValueError, "9"):
            runner.act(observation())

    def test_reset_and_timing_log(self):
        loader = FakeLoader(steps=2)
        runner = PickPolicy("p", loader=loader, clock=Clock())
        runner.load()
        runner.reset()
        with self.assertLogs("robot.policy.pick_policy", "INFO") as logs:
            for _ in range(4):
                runner.act(observation())
        self.assertEqual(loader.resets, 1)
        self.assertIn("first inference 50.0 ms", logs.output[0])
        self.assertEqual(runner.timing.count, 2)  # frames 1 and 3 ran the model (2 steps per inference)


class DeviceTests(unittest.TestCase):
    def test_auto_prefers_mps(self):
        self.assertEqual(resolve_device("auto", lambda: True), "mps")
        self.assertEqual(resolve_device("auto", lambda: False), "cpu")
        self.assertEqual(resolve_device("cuda", lambda: True), "cuda")

    def test_warm_up_frame_matches_the_policy_inputs(self):
        frame = zero_frame({"observation.state": (9,), "observation.images.wrist": (3, 480, 640)})
        self.assertEqual((frame["observation.state"].shape, frame["observation.state"].dtype), ((9,), np.float32))
        self.assertEqual((frame["observation.images.wrist"].shape, frame["observation.images.wrist"].dtype),
                         ((480, 640, 3), np.uint8))


class RealConfigTests(unittest.TestCase):
    def test_act_config_takes_the_action_horizon(self):
        code = ("import sys\n"
                "try:\n    from lerobot.policies.act.configuration_act import ACTConfig\n"
                "except ImportError:\n    sys.exit(77)\n"
                "from robot.policy.lerobot_policy import _with_overrides\n"
                "config = _with_overrides(ACTConfig(), 'ckpt', 'cpu', 10)\n"
                "assert (config.n_action_steps, config.chunk_size, config.device) == (10, 100, 'cpu'), config\n"
                "assert _with_overrides(ACTConfig(chunk_size=5, n_action_steps=5), 'c', 'cpu', 10).n_action_steps == 5\n")
        env = {**os.environ, "PYTHONPATH": SRC}
        result = subprocess.run([sys.executable, "-c", code], env=env, capture_output=True, timeout=300)
        if result.returncode == 77:
            self.skipTest("LeRobot is not installed")
        self.assertEqual(result.returncode, 0, result.stderr.decode())


if __name__ == "__main__":
    unittest.main()
