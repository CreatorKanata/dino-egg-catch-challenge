"""tests/robot/test_egg_size.py: Hardware-free checks of the Auto Catch egg-size precondition.

classify_size() is stdlib-only (no OpenCV), so each size class and the window bounds are
verified directly with EggDetection records.
"""

import unittest

from robot.config import AUTO_CATCH_MAX_EGG_H, AUTO_CATCH_MIN_EGG_H
from robot.vision.egg_size import SIZE_CLASSES, EggDetection, classify_size


def egg(h):
    return EggDetection(cx=0.5, cy=0.5, w=h, h=h, color="green", spots=3, area_px=1000)


class ClassifySizeTests(unittest.TestCase):
    def test_each_class(self):
        cases = ((None, "none"), (egg(0.05), "too_small"), (egg(0.5), "ok"), (egg(0.95), "too_large"))
        for det, expected in cases:
            with self.subTest(expected=expected):
                self.assertEqual(classify_size(det), expected)
                self.assertIn(expected, SIZE_CLASSES)

    def test_bounds_count_as_ok(self):
        self.assertEqual(classify_size(egg(AUTO_CATCH_MIN_EGG_H)), "ok")
        self.assertEqual(classify_size(egg(AUTO_CATCH_MAX_EGG_H)), "ok")

    def test_custom_window(self):
        self.assertEqual(classify_size(egg(0.5), min_h=0.6, max_h=0.9), "too_small")
        self.assertEqual(classify_size(egg(0.5), min_h=0.1, max_h=0.4), "too_large")


if __name__ == "__main__":
    unittest.main()
