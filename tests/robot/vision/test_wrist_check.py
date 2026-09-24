"""tests/robot/vision/test_wrist_check.py: Synthetic-frame checks of the wrist-view egg check (Phase 3 step 1).

Frames are 640x480 like the wrist camera, drawn with cv2 on the blue tarp color used by the egg
detector tests: a close-up egg filling most of the frame gives a full detection, an egg cut by the
border with only two spots visible gives a partial one (the spot-cluster fallback), a single
ringed spot or an empty frame gives none, and `inspect --wrist` prints the result. Runs in its own
interpreter (see tests/robot/test_vision_suite.py) because OpenCV and pygame must not share a
process on macOS.
"""

import contextlib
import io
from pathlib import Path
import shutil
import tempfile
import unittest

import cv2
import numpy as np

from robot.vision import inspect as inspect_tool
from robot.vision.egg_size import WristView
from robot.vision.wrist_check import egg_in_wrist_view

WIDTH, HEIGHT = 640, 480
TARP = (110, 65, 30)  # BGR, HSV (107, 185, 110): neither white body nor any spot class
WHITE = (245, 245, 245)
GREEN = (40, 160, 60)
RED = (40, 40, 220)
SPOT_LAYOUT = ((-0.55, 0.05), (0.55, -0.1), (0.0, -0.6), (0.0, 0.55), (0.05, 0.0))  # x a, y b


def background():
    return np.full((HEIGHT, WIDTH, 3), TARP, dtype=np.uint8)


def close_up_egg(spot=GREEN):
    """An egg filling most of the frame (semi-axes 300 x 215), five spots of diameter ~egg height / 4."""
    frame, center, axes = background(), (320, 250), (300, 215)
    cv2.ellipse(frame, center, axes, 0, 0, 360, WHITE, -1)
    for fx, fy in SPOT_LAYOUT:
        cv2.circle(frame, (center[0] + int(fx * axes[0]), center[1] + int(fy * axes[1])), axes[1] // 4, spot, -1)
    return frame


def partial_egg():
    """The right part of an egg much larger than the view (it enters from the left border) with two
    spots visible: too tall for its spots (scale), so only the spot-cluster fallback finds it."""
    frame = background()
    cv2.ellipse(frame, (-100, 240), (400, 300), 0, 0, 360, WHITE, -1)
    for center in ((120, 180), (150, 300)):
        cv2.circle(frame, center, 30, RED, -1)
    return frame


class WristCheckTests(unittest.TestCase):
    def test_close_up_egg_is_a_full_detection(self):
        view = egg_in_wrist_view(close_up_egg())
        self.assertIsInstance(view, WristView)
        self.assertEqual((view.color, view.partial), ("green", False))
        self.assertAlmostEqual(view.cx, 0.5, delta=0.05)
        self.assertAlmostEqual(view.cy, 0.52, delta=0.05)

    def test_egg_cut_by_the_border_with_two_spots_is_partial(self):
        view = egg_in_wrist_view(partial_egg())
        self.assertIsInstance(view, WristView)
        self.assertEqual((view.color, view.partial), ("red", True))
        self.assertLess(view.cx, 0.5)  # the cluster window sits in the left half

    def test_no_egg_and_a_single_spot_give_none(self):
        self.assertIsNone(egg_in_wrist_view(background()))
        single = background()
        cv2.ellipse(single, (-100, 240), (400, 300), 0, 0, 360, WHITE, -1)
        cv2.circle(single, (150, 240), 30, GREEN, -1)  # one ringed spot: below WRIST_MIN_SPOT_CLUSTER
        self.assertIsNone(egg_in_wrist_view(single))

    def test_rejects_non_color_frames(self):
        with self.assertRaises(ValueError):
            egg_in_wrist_view(np.zeros((HEIGHT, WIDTH), dtype=np.uint8))


class InspectWristTests(unittest.TestCase):
    def setUp(self):
        self.folder = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.folder, ignore_errors=True)

    def run_tool(self, *args):
        out = io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(io.StringIO()):
            code = inspect_tool.main([str(arg) for arg in args])
        return code, out.getvalue()

    def test_wrist_flag_prints_the_check_result(self):
        for name, frame, expected in (("full", close_up_egg(), "wrist: green egg (full)"),
                                      ("partial", partial_egg(), "wrist: red egg (partial)"),
                                      ("empty", background(), "wrist: no egg")):
            with self.subTest(name=name):
                image = self.folder / f"{name}-wrist.png"
                cv2.imwrite(str(image), frame)
                code, output = self.run_tool(image, "--wrist")
                self.assertEqual(code, 0)
                self.assertIn(expected, output)


if __name__ == "__main__":
    unittest.main()
