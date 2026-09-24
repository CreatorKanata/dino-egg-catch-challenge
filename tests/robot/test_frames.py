"""tests/robot/test_frames.py: Checks of the Pi camera RGB -> BGR conversion after observe().

LeKiwiClient returns the front and wrist frames RGB-ordered; normalize_observation_frames must
swap them to BGR once, leave other entries (the top frame, scalars, missing frames) alone, and
never modify its input. numpy only, so this runs in the main (OpenCV-free) test process.
"""

import unittest

import numpy as np

from robot.vision.frames import normalize_observation_frames, rgb_to_bgr

RGB = np.array([[[255, 0, 0], [0, 255, 0]], [[0, 0, 255], [10, 20, 30]]], dtype=np.uint8)  # 2x2
BGR = np.array([[[0, 0, 255], [0, 255, 0]], [[255, 0, 0], [30, 20, 10]]], dtype=np.uint8)


class FramesTests(unittest.TestCase):
    def test_rgb_to_bgr_swaps_channels_into_a_new_contiguous_array(self):
        out = rgb_to_bgr(RGB)
        np.testing.assert_array_equal(out, BGR)
        self.assertTrue(out.flags["C_CONTIGUOUS"])
        self.assertFalse(np.shares_memory(out, RGB))

    def test_rgb_to_bgr_rejects_non_color_frames(self):
        with self.assertRaises(ValueError):
            rgb_to_bgr(np.zeros((2, 2), dtype=np.uint8))

    def test_front_and_wrist_swapped_top_and_scalars_untouched(self):
        top = RGB.copy()
        observation = {"front": RGB, "wrist": RGB, "top": top, "x.vel": 0.1}
        out = normalize_observation_frames(observation, "rgb")
        np.testing.assert_array_equal(out["front"], BGR)
        np.testing.assert_array_equal(out["wrist"], BGR)
        self.assertIs(out["top"], top)
        self.assertEqual(out["x.vel"], 0.1)
        self.assertIsNot(out, observation)
        np.testing.assert_array_equal(observation["front"], RGB)  # input unchanged

    def test_missing_frames_stay_missing(self):
        out = normalize_observation_frames({"front": None, "x.vel": 0.0}, "rgb")
        self.assertEqual(out, {"front": None, "x.vel": 0.0})

    def test_bgr_order_disables_the_conversion(self):
        out = normalize_observation_frames({"front": RGB}, "bgr")
        self.assertIs(out["front"], RGB)

    def test_unknown_order_is_rejected(self):
        with self.assertRaises(ValueError):
            normalize_observation_frames({"front": RGB}, "yuv")


if __name__ == "__main__":
    unittest.main()
