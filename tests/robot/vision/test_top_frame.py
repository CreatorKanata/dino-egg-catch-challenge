"""tests/robot/vision/test_top_frame.py: Checks of the overhead frame downscale (OpenCV resize).

A 1280x720 16:9 frame must become 960x540 with the aspect kept and the input untouched; odd
sizes keep their aspect within a pixel. Runs in its own interpreter with OpenCV (see
tests/robot/test_vision_suite.py).
"""

import unittest

import numpy as np

from robot.config import TOP_CAMERA_HEIGHT, TOP_CAMERA_WIDTH, TOP_DISPLAY_WIDTH
from robot.vision.top_frame import downscale_to_width


class DownscaleTests(unittest.TestCase):
    def test_720p_becomes_960x540_and_input_is_untouched(self):
        frame = np.zeros((TOP_CAMERA_HEIGHT, TOP_CAMERA_WIDTH, 3), dtype=np.uint8)
        frame[:, :640] = (255, 0, 0)  # left half blue (BGR)
        original = frame.copy()
        out = downscale_to_width(frame)
        self.assertEqual(out.shape, (540, TOP_DISPLAY_WIDTH, 3))
        np.testing.assert_array_equal(frame, original)
        self.assertEqual(tuple(out[270, 100]), (255, 0, 0))  # channel order kept
        self.assertEqual(tuple(out[270, 900]), (0, 0, 0))

    def test_other_sizes_keep_their_aspect(self):
        out = downscale_to_width(np.zeros((1080, 1920, 3), dtype=np.uint8), 640)
        self.assertEqual(out.shape, (360, 640, 3))
        odd = downscale_to_width(np.zeros((481, 1001, 3), dtype=np.uint8), 500)
        self.assertEqual(odd.shape[:2], (240, 500))


if __name__ == "__main__":
    unittest.main()
