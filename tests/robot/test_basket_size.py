"""tests/robot/test_basket_size.py: Checks of the basket size precondition and Auto Release config.

classify_basket() is stdlib-only, so each size class and its bound are verified directly; the
basket is never "too close". The Auto Release config guards are checked too: the staff keys are
distinct single characters that no KachiButton phrase contains (the "h" of "Thx" and "Hi!" would
fire a home key "h"), and the playback speed, tolerances, and target are consistent.
"""

import unittest

from robot import config
from robot.vision import config_vision
from robot.vision.basket_size import BASKET_SIZE_CLASSES, BasketDetection, classify_basket


def basket(w):
    return BasketDetection(cx=0.5, cy=0.4, w=w, h=0.6, area_fraction=0.3, fill=0.6)


class ClassifyBasketTests(unittest.TestCase):
    def test_each_class_and_the_bound(self):
        cases = ((None, "none"), (basket(0.1), "too_small"), (basket(config_vision.AUTO_RELEASE_MIN_BASKET_W), "ok"),
                 (basket(0.8), "ok"), (basket(1.0), "ok"))  # filling the frame is the release position
        for det, expected in cases:
            with self.subTest(expected=expected, det=det):
                self.assertEqual(classify_basket(det), expected)
                self.assertIn(expected, BASKET_SIZE_CLASSES)
        self.assertEqual(classify_basket(basket(0.5), min_w=0.6), "too_small")


class ReleaseConfigTests(unittest.TestCase):
    def test_staff_keys_never_collide_with_phrases(self):
        keys = (config.CAPTURE_KEY, config.HOME_KEY, config.RECORD_KEY)
        self.assertEqual(len(set(keys)), 3)
        for key in keys:
            with self.subTest(key=key):
                self.assertEqual(len(key), 1)
                self.assertFalse(any(key.lower() in phrase.lower() for phrase, _ in config.KACHI_PHRASES))
        self.assertTrue(any("h" in phrase.lower() for phrase, _ in config.KACHI_PHRASES))  # why not "h"

    def test_release_values_are_consistent(self):
        self.assertTrue(0 < config.RELEASE_PLAYBACK_SPEED <= 2)
        self.assertEqual(config.RECORD_MAX_S, 30.0)
        self.assertLess(config_vision.AUTO_RELEASE_MIN_BASKET_W,
                        config_vision.RELEASE_TARGET_W - config_vision.RELEASE_TOL_W_FAR)
        self.assertEqual((config.HOME_POSE_PATH, config.RELEASE_MOTION_PATH),
                         ("data/arm/home_pose.json", "data/arm/release_motion.json"))
        self.assertTrue((config.REPO_ROOT / "src" / "robot" / "config.py").is_file())


if __name__ == "__main__":
    unittest.main()
