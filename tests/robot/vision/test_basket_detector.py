"""tests/robot/vision/test_basket_detector.py: Synthetic-frame checks of the pink basket detector.

Frames are drawn with cv2 (640x480, blue tarp-colored background). A large dark, desaturated pink
region (the basket at the release position) is found with its bbox; a red-spotted egg alone gives
no basket (its spots are pink-ish but small), and with a large pink region next to it the basket
bbox is the region's; small or sparse pink areas are rejected; inspect `--basket` prints the
detection. Runs in its own interpreter (see tests/robot/test_vision_suite.py).
"""

import contextlib
import io
from pathlib import Path
import shutil
import tempfile
import time
import unittest

import cv2
import numpy as np

from robot.vision import inspect as inspect_tool
from robot.vision.basket_detector import BasketParams, basket_mask, detect_basket
from robot.vision.basket_size import classify_basket

WIDTH, HEIGHT = 640, 480
TARP = (110, 65, 30)  # BGR: HSV (107, 185, 110), not pink
DARK_PINK = (40, 30, 48)  # BGR: HSV ~(165, 96, 48): the basket at the release position (dark, desaturated)
WHITE = (245, 245, 245)
RED_SPOT = (59, 43, 200)  # BGR: HSV ~(177, 200, 200): a red egg spot, inside the basket hue range
TOLERANCE = 6 / WIDTH  # normalized, a few pixels (half-scale morphology)


def background():
    return np.full((HEIGHT, WIDTH, 3), TARP, dtype=np.uint8)


def red_egg(frame, center=(200, 250), axes=(110, 85)):
    cv2.ellipse(frame, center, axes, 0, 0, 360, WHITE, -1)
    for fx, fy in ((-0.55, 0.05), (0.55, -0.1), (0.0, -0.6), (0.0, 0.55), (0.05, 0.0)):
        cv2.circle(frame, (center[0] + int(fx * axes[0]), center[1] + int(fy * axes[1])), axes[1] // 4, RED_SPOT, -1)
    return frame


class DetectBasketTests(unittest.TestCase):
    def assert_box(self, det, x0, y0, x1, y1):
        want = ((x0 + x1 + 1) / 2 / WIDTH, (y0 + y1 + 1) / 2 / HEIGHT, (x1 - x0 + 1) / WIDTH, (y1 - y0 + 1) / HEIGHT)
        for got, expected in zip((det.cx, det.cy, det.w, det.h), want):
            self.assertLessEqual(abs(got - expected), TOLERANCE, (det, want))

    def test_large_dark_pink_region_is_the_basket(self):
        frame = background()
        cv2.rectangle(frame, (70, 30), (570, 360), DARK_PINK, -1)
        det = detect_basket(frame)
        self.assert_box(det, 70, 30, 570, 360)
        self.assertGreater(det.fill, 0.95)
        self.assertEqual(classify_basket(det), "ok")

    def test_red_spotted_egg_alone_is_no_basket(self):
        self.assertIsNone(detect_basket(red_egg(background())))

    def test_egg_plus_pink_region_reports_the_region(self):
        frame = background()
        cv2.rectangle(frame, (330, 60), (620, 330), DARK_PINK, -1)
        det = detect_basket(red_egg(frame))
        self.assert_box(det, 330, 60, 620, 330)

    def test_small_or_sparse_pink_is_rejected(self):
        small = background()
        cv2.rectangle(small, (300, 200), (400, 260), DARK_PINK, -1)  # 2 % of the frame
        self.assertIsNone(detect_basket(small))
        sparse = background()
        cv2.line(sparse, (20, 20), (620, 460), DARK_PINK, 30)  # big bbox, thin diagonal: low fill
        self.assertIsNone(detect_basket(sparse))
        self.assertIsNotNone(detect_basket(sparse, BasketParams(min_fill=0.05)))

    def test_mask_is_half_size_and_detection_is_fast(self):
        frame = background()
        cv2.rectangle(frame, (70, 30), (570, 360), DARK_PINK, -1)
        self.assertEqual(basket_mask(frame).shape, (HEIGHT // 2, WIDTH // 2))
        detect_basket(frame)
        started = time.perf_counter()
        for _ in range(20):
            detect_basket(frame)
        self.assertLess((time.perf_counter() - started) / 20, 0.010)  # ~1.3 ms measured; loose bound

    def test_rejects_non_color_frames(self):
        with self.assertRaises(ValueError):
            detect_basket(np.zeros((HEIGHT, WIDTH), dtype=np.uint8))


class InspectBasketTests(unittest.TestCase):
    def test_basket_flag_prints_the_detection(self):
        folder = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, folder, ignore_errors=True)
        frame = background()
        cv2.rectangle(frame, (70, 30), (570, 360), DARK_PINK, -1)
        image = folder / "front.png"
        cv2.imwrite(str(image), frame)
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(inspect_tool.main([str(image), "--basket"]), 0)
        self.assertIn("basket: cx=0.5", out.getvalue())
        self.assertIn("size=ok", out.getvalue())
        self.assertTrue((folder / "front-detected.png").exists())
        with contextlib.redirect_stdout(io.StringIO()) as plain:
            inspect_tool.main([str(image)])
        self.assertNotIn("basket:", plain.getvalue())


if __name__ == "__main__":
    unittest.main()
