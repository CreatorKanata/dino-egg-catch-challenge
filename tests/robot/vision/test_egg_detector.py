"""tests/robot/vision/test_egg_detector.py: Synthetic-frame checks of the front-camera egg detector.

Frames are drawn with cv2.ellipse/cv2.circle (640x480, brown background): a white ellipse with
green, blue, or red spots (red on both hue ends) must yield one detection of that color with a
bbox close to the drawn ellipse; spotless or too-small eggs and a plain background yield none;
two eggs come back largest first; an egg in front of a pink basket-colored blob stays one green
egg; a non-convex white L is rejected; an egg cut by the frame border is still found. Runs in
its own interpreter (see tests/robot/test_vision_suite.py) because OpenCV and pygame must not
share a process on macOS.
"""

import unittest

import cv2
import numpy as np

from robot.vision.egg_detector import DetectorParams, EggDetection, best_egg, detect_eggs

WIDTH, HEIGHT = 640, 480
BROWN = (40, 70, 110)  # BGR: H ~13, S ~160 -> neither white body nor any spot color
WHITE = (245, 245, 245)
PINK = (110, 80, 150)  # BGR: H ~167, S ~119, V 150 -> inside BASKET_HSV
SPOT_BGR = {"green": (40, 160, 60), "blue": (200, 60, 30), "red_low": (40, 40, 220), "red_high": (90, 30, 220)}
TOLERANCE_PX = 4


def background():
    return np.full((HEIGHT, WIDTH, 3), BROWN, dtype=np.uint8)


def draw_egg(frame, center, axes, spot=None, spot_radius=18):
    """White ellipse with three spots of `spot` color (None: no spots); returns the frame."""
    cv2.ellipse(frame, center, axes, 0, 0, 360, WHITE, -1)
    if spot is not None:
        cx, cy = center
        for dx, dy in ((-axes[0] // 2, 0), (axes[0] // 2, -axes[1] // 3), (0, axes[1] // 2)):
            cv2.circle(frame, (cx + dx, cy + dy), spot_radius, SPOT_BGR[spot], -1)
    return frame


def expected_box(center, axes):
    """Normalized (cx, cy, w, h) of the drawn ellipse's bbox."""
    return center[0] / WIDTH, center[1] / HEIGHT, (2 * axes[0] + 1) / WIDTH, (2 * axes[1] + 1) / HEIGHT


class DetectEggsTests(unittest.TestCase):
    def assert_box(self, det, center, axes):
        for got, want, scale in zip((det.cx, det.cy, det.w, det.h), expected_box(center, axes),
                                    (WIDTH, HEIGHT, WIDTH, HEIGHT)):
            self.assertLessEqual(abs(got - want) * scale, TOLERANCE_PX, (det, center, axes))

    def test_green_egg_one_detection_with_tight_bbox(self):
        center, axes = (330, 260), (120, 150)
        detections = detect_eggs(draw_egg(background(), center, axes, "green"))
        self.assertEqual(len(detections), 1)
        det = detections[0]
        self.assertIsInstance(det, EggDetection)
        self.assertEqual((det.color, det.spots), ("green", 3))
        self.assert_box(det, center, axes)
        self.assertGreater(det.area_px, 50000)

    def test_blue_and_red_variants_on_both_hue_ends(self):
        for spot, color in (("blue", "blue"), ("red_low", "red"), ("red_high", "red")):
            with self.subTest(spot=spot):
                center, axes = (300, 240), (100, 130)
                detections = detect_eggs(draw_egg(background(), center, axes, spot))
                self.assertEqual([det.color for det in detections], [color])
                self.assert_box(detections[0], center, axes)

    def test_spotless_white_ellipse_is_not_an_egg(self):
        self.assertEqual(detect_eggs(draw_egg(background(), (320, 240), (100, 130))), ())

    def test_small_egg_below_area_threshold(self):
        frame = draw_egg(background(), (320, 240), (15, 20), "green", spot_radius=5)
        self.assertEqual(detect_eggs(frame), ())
        self.assertIsNone(best_egg(frame))

    def test_two_eggs_sorted_by_area(self):
        frame = draw_egg(background(), (150, 240), (60, 80), "red_low")
        frame = draw_egg(frame, (450, 240), (110, 140), "green")
        detections = detect_eggs(frame)
        self.assertEqual([det.color for det in detections], ["green", "red"])
        self.assertGreater(detections[0].area_px, detections[1].area_px)
        self.assertEqual(best_egg(frame), detections[0])

    def test_background_only_is_empty(self):
        self.assertEqual(detect_eggs(background()), ())

    def test_aspect_filter_rejects_a_long_white_bar(self):
        frame = background()
        cv2.rectangle(frame, (20, 200), (620, 260), WHITE, -1)
        cv2.circle(frame, (320, 230), 20, SPOT_BGR["green"], -1)
        self.assertEqual(detect_eggs(frame), ())

    def test_min_spots_parameter(self):
        frame = draw_egg(background(), (320, 240), (100, 130), "green")
        self.assertEqual(detect_eggs(frame, DetectorParams(min_spots=4)), ())

    def test_egg_in_front_of_pink_basket_is_one_green_egg(self):
        frame = background()
        cv2.rectangle(frame, (250, 60), (639, 330), PINK, -1)  # basket behind and right of the egg
        center, axes = (260, 250), (110, 120)
        detections = detect_eggs(draw_egg(frame, center, axes, "green"))
        self.assertEqual([det.color for det in detections], ["green"])
        self.assert_box(detections[0], center, axes)
        self.assertFalse(detections[0].touches_border)

    def test_pink_blob_alone_is_not_an_egg(self):
        frame = background()
        cv2.ellipse(frame, (320, 240), (120, 150), 0, 0, 360, PINK, -1)
        self.assertEqual(detect_eggs(frame), ())

    def test_non_convex_white_l_is_rejected(self):
        frame = background()
        cv2.rectangle(frame, (100, 100), (300, 160), WHITE, -1)
        cv2.rectangle(frame, (100, 100), (160, 380), WHITE, -1)
        for center in ((130, 250), (130, 330), (230, 130)):
            cv2.circle(frame, center, 18, SPOT_BGR["green"], -1)
        self.assertEqual(detect_eggs(frame), ())

    def test_egg_cut_by_the_frame_border_is_found(self):
        frame = background()
        cv2.ellipse(frame, (40, 240), (100, 130), 0, 0, 360, WHITE, -1)
        for center in ((70, 190), (90, 300), (25, 250)):
            cv2.circle(frame, center, 18, SPOT_BGR["green"], -1)
        detections = detect_eggs(frame)
        self.assertEqual(len(detections), 1)
        det = detections[0]
        self.assertEqual((det.color, det.touches_border), ("green", True))
        self.assertLessEqual(abs(det.w * WIDTH - 141), TOLERANCE_PX)  # visible part: x 0..140
        self.assertGreater(det.solidity, 0.95)

    def test_ellipse_fill_applies_only_away_from_the_border(self):
        strict = DetectorParams(ellipse_fill_range=(2.0, 3.0))  # no real outline can pass
        inside = draw_egg(background(), (320, 240), (100, 130), "green")
        self.assertEqual(detect_eggs(inside, strict), ())
        cut = draw_egg(background(), (60, 240), (100, 130), "green")
        self.assertEqual([det.touches_border for det in detect_eggs(cut, strict)], [True])

    def test_solidity_is_recorded(self):
        det = detect_eggs(draw_egg(background(), (320, 240), (100, 130), "green"))[0]
        self.assertGreater(det.solidity, 0.95)
        self.assertLessEqual(det.solidity, 1.0)

    def test_rejects_non_color_frames(self):
        with self.assertRaises(ValueError):
            detect_eggs(np.zeros((HEIGHT, WIDTH), dtype=np.uint8))


if __name__ == "__main__":
    unittest.main()
