"""tests/robot/vision/test_wrist_check.py: Synthetic-frame checks of the close-up wrist-view egg check.

Frames are 640x480 like the wrist camera, drawn with cv2 on the blue tarp color: giant red spots on
white, cut by the border, are a (partial) egg; the white gripper parts (large white shapes at the
bottom and top right, no spots), a teal gripper part among white, the pink basket edge (right), and
an empty frame are not; a spot-colored disk without a white ring is not; `inspect --wrist` lists
the spot candidates and the verdict. Runs in its own interpreter (see tests/robot/test_vision_suite.py)
because OpenCV and pygame must not share a process on macOS.
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
from robot.vision.wrist_check import egg_in_wrist_view, wrist_spots

WIDTH, HEIGHT = 640, 480
TARP = (110, 65, 30)  # BGR, HSV (107, 185, 110): neither white body nor any spot class
WHITE = (245, 245, 245)
RED = (40, 40, 220)
GREEN = (40, 160, 60)
TEAL = (120, 150, 30)  # BGR, HSV ~ (80, 204, 150): the teal gripper part, "green" by hue
PINK = (150, 90, 220)  # BGR, HSV ~ (171, 150, 220): the pink basket edge, red-ish by hue


def background():
    return np.full((HEIGHT, WIDTH, 3), TARP, dtype=np.uint8)


def gripper(frame):
    """White gripper parts without spots: teeth at the bottom center, a wedge at the top right."""
    cv2.rectangle(frame, (230, 420), (290, 479), WHITE, -1)
    cv2.rectangle(frame, (350, 450), (400, 479), WHITE, -1)
    cv2.fillPoly(frame, [np.array([[420, 180], [639, 60], [639, 230], [470, 220]], dtype=np.int32)], WHITE)
    return frame


def near_egg():
    """The close-up egg: a white ellipse cut by the left and bottom borders with giant red spots."""
    frame = gripper(background())
    cv2.ellipse(frame, (120, 330), (180, 170), 0, 0, 360, WHITE, -1)
    for center in ((190, 280), (60, 350), (130, 180)):
        cv2.circle(frame, center, 50, RED, -1)
    return frame


class WristCheckTests(unittest.TestCase):
    def test_near_egg_cut_by_the_border_is_a_partial_red_egg(self):
        view = egg_in_wrist_view(near_egg())
        self.assertIsInstance(view, WristView)
        self.assertEqual((view.color, view.partial), ("red", True))
        self.assertLess(view.cx, 0.5)
        self.assertGreater(view.cy, 0.4)

    def test_egg_inside_the_view_is_not_partial(self):
        frame = background()
        cv2.ellipse(frame, (320, 240), (200, 160), 0, 0, 360, WHITE, -1)
        cv2.circle(frame, (320, 240), 50, GREEN, -1)
        view = egg_in_wrist_view(frame)
        self.assertEqual((view.color, view.partial), ("green", False))
        self.assertAlmostEqual(view.cx, 0.5, delta=0.02)

    def test_gripper_parts_and_empty_frames_are_no_egg(self):
        self.assertIsNone(egg_in_wrist_view(background()))
        self.assertIsNone(egg_in_wrist_view(gripper(background())))

    def test_teal_gripper_part_among_white_is_no_egg(self):
        frame = gripper(background())
        cv2.ellipse(frame, (590, 70), (50, 90), 0, 0, 360, TEAL, -1)  # elongated, not a spot disk
        self.assertIsNone(egg_in_wrist_view(frame))

    def test_pink_basket_edge_is_no_egg(self):
        frame = gripper(background())
        cv2.ellipse(frame, (600, 340), (80, 130), 0, 0, 360, PINK, -1)  # cut by the right border, no white ring
        self.assertIsNone(egg_in_wrist_view(frame))

    def test_spot_colored_disk_without_a_white_ring_is_no_egg(self):
        frame = background()
        cv2.circle(frame, (320, 240), 60, RED, -1)
        self.assertIsNone(egg_in_wrist_view(frame))
        self.assertTrue(all(not spot.accepted for spot in wrist_spots(frame)))

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

    def test_wrist_flag_lists_spots_and_prints_the_verdict(self):
        for name, frame, expected in (("near", near_egg(), "wrist: red egg (partial)"),
                                      ("empty", gripper(background()), "wrist: no egg")):
            with self.subTest(name=name):
                image = self.folder / f"{name}-wrist.png"
                cv2.imwrite(str(image), frame)
                code, output = self.run_tool(image, "--wrist")
                self.assertEqual(code, 0)
                self.assertIn(expected, output)
        self.assertIn("ACCEPTED", self.run_tool(self.folder / "near-wrist.png", "--wrist")[1])


if __name__ == "__main__":
    unittest.main()
