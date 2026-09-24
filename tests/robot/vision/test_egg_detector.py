"""tests/robot/vision/test_egg_detector.py: Synthetic-frame checks of the spot-anchored egg detector.

Frames are drawn with cv2 (640x480, blue tarp-colored background, no spot class) using the eggs'
measured proportions: a white ellipse wider than tall with five spots of diameter ~1/4 of the egg
height. One egg of each spot color (green, red on both hue ends, orange) gives one detection with a
tight bbox and a fitted ellipse; a blue-spotted egg (blue dropped 2026-09-24) gives none;
spotless shapes, pink blobs, and white bars give none; thin glint streaks touching an egg do not
widen its box; two distant eggs give two clusters; eggs cut by the border and notched eggs are
found; a red-spotted egg in front of the pink basket stays one red egg; every candidate reports
the rule that rejected it. Runs in its own interpreter (see
tests/robot/test_vision_suite.py) because OpenCV and pygame must not share a process on macOS.
"""

import unittest

import cv2
import numpy as np

from robot.vision.egg_detector import DetectorParams, EggDetection, best_egg, detect_eggs, inspect_candidates

WIDTH, HEIGHT = 640, 480
BROWN = (110, 65, 30)  # BGR, HSV (107, 185, 110): blue tarp color -> neither white body nor any spot class
WHITE = (245, 245, 245)
PINK = (110, 80, 150)  # BGR: H ~167, S ~119, V 150 -> inside BASKET_HSV
# red_low is H 0, red_high H 177 (both ends of the red hue range), orange H 15
SPOT_BGR = {"green": (40, 160, 60), "orange": (30, 125, 220), "red_low": (40, 40, 220), "red_high": (59, 43, 200)}
BASKET_BGR = ((38, 29, 60), (29, 18, 60))  # HSV (171, 132, 60) and (172, 179, 60): the real basket's pink
SPOT_LAYOUT = ((-0.55, 0.05), (0.55, -0.1), (0.0, -0.6), (0.0, 0.55), (0.05, 0.0))  # x a, y b
TOLERANCE_PX = 4
HSV = DetectorParams(spot_detector="hsv")
TARP_BGR = (72, 44, 26)  # HSV ~ (105, 162, 72): the blue tarp, and the (dropped) blue egg's spots
BLUE_SPOT_BGR = (70, 30, 10)  # HSV ~ (104, 219, 70): dark blue spot, the same hue as the tarp


def background():
    return np.full((HEIGHT, WIDTH, 3), BROWN, dtype=np.uint8)


def draw_egg(frame, center, axes, spot=None):
    """White ellipse (semi-axes a, b) with five spots of radius b / 4 (spot d = egg height / 4)."""
    cv2.ellipse(frame, center, axes, 0, 0, 360, WHITE, -1)
    if spot is not None:
        for fx, fy in SPOT_LAYOUT:
            cv2.circle(frame, (center[0] + int(fx * axes[0]), center[1] + int(fy * axes[1])), axes[1] // 4,
                       SPOT_BGR[spot], -1)
    return frame


def expected_box(center, axes):
    return center[0] / WIDTH, center[1] / HEIGHT, (2 * axes[0] + 1) / WIDTH, (2 * axes[1] + 1) / HEIGHT


class DetectEggsTests(unittest.TestCase):
    def assert_box(self, det, center, axes):
        for got, want, scale in zip((det.cx, det.cy, det.w, det.h), expected_box(center, axes),
                                    (WIDTH, HEIGHT, WIDTH, HEIGHT)):
            self.assertLessEqual(abs(got - want) * scale, TOLERANCE_PX, (det, center, axes))

    def test_green_egg_one_detection_with_tight_bbox_and_ellipse(self):
        center, axes = (330, 250), (130, 100)
        detections = detect_eggs(draw_egg(background(), center, axes, "green"))
        self.assertEqual(len(detections), 1)
        det = detections[0]
        self.assertIsInstance(det, EggDetection)
        self.assertEqual((det.color, det.spots, det.touches_border), ("green", 5, False))
        self.assert_box(det, center, axes)
        ex, ey, ea, eb, angle = det.ellipse
        self.assertLessEqual(abs(ex * WIDTH - center[0]) + abs(ey * HEIGHT - center[1]), 4)
        axes_px = sorted((ea * WIDTH, eb * HEIGHT))
        self.assertLessEqual(abs(axes_px[0] - 200) + abs(axes_px[1] - 260), 8)
        self.assertGreater(det.solidity, 0.95)

    def test_orange_and_red_variants_on_both_hue_ends(self):
        for spot, color in (("orange", "orange"), ("red_low", "red"), ("red_high", "red")):
            with self.subTest(spot=spot):
                center, axes = (300, 240), (120, 90)
                detections = detect_eggs(draw_egg(background(), center, axes, spot))
                self.assertEqual([det.color for det in detections], [color])
                self.assert_box(detections[0], center, axes)

    def test_shapes_without_spots_are_not_eggs(self):
        frame = draw_egg(background(), (200, 240), (100, 80))
        cv2.ellipse(frame, (480, 240), (100, 120), 0, 0, 360, PINK, -1)
        self.assertEqual(detect_eggs(frame), ())

    def test_small_egg_below_limits(self):
        frame = draw_egg(background(), (320, 240), (20, 15), "green")
        self.assertEqual(detect_eggs(frame), ())
        self.assertIsNone(best_egg(frame))

    def test_two_distant_eggs_are_two_clusters_sorted_by_area(self):
        frame = draw_egg(background(), (140, 240), (80, 62), "red_low")
        frame = draw_egg(frame, (450, 240), (130, 100), "green")
        self.assertEqual(sum(candidate.rejected is None for candidate in inspect_candidates(frame)), 2)
        detections = detect_eggs(frame)
        self.assertEqual([det.color for det in detections], ["green", "red"])
        self.assertGreater(detections[0].area_px, detections[1].area_px)
        self.assertEqual(best_egg(frame), detections[0])

    def test_glint_streaks_touching_the_egg_do_not_widen_the_box(self):
        center, axes = (300, 240), (120, 90)
        frame = draw_egg(background(), center, axes, "green")
        for y in range(150, 340, 25):  # thin white wrinkle glints running into the egg from the right
            cv2.line(frame, (390, y), (630, y + 30), WHITE, 5)
        detections = detect_eggs(frame)
        self.assertEqual(len(detections), 1)
        self.assert_box(detections[0], center, axes)

    def test_egg_in_front_of_pink_basket_is_one_green_egg(self):
        frame = background()
        cv2.rectangle(frame, (250, 60), (639, 330), PINK, -1)
        center, axes = (260, 250), (130, 100)
        detections = detect_eggs(draw_egg(frame, center, axes, "green"))
        self.assertEqual([det.color for det in detections], ["green"])
        self.assert_box(detections[0], center, axes)

    def test_red_egg_in_front_of_the_pink_basket_is_one_red_egg(self):
        frame = background()
        cv2.rectangle(frame, (200, 40), (639, 200), BASKET_BGR[0], -1)  # S 132: basket range 2 / below red S
        cv2.rectangle(frame, (200, 200), (639, 380), BASKET_BGR[1], -1)  # S 179 at H 172: basket range 1
        center, axes = (280, 250), (130, 100)
        for spot in ("red_low", "red_high"):
            with self.subTest(spot=spot):
                detections = detect_eggs(draw_egg(frame.copy(), center, axes, spot))
                self.assertEqual([det.color for det in detections], ["red"])
                self.assert_box(detections[0], center, axes)

    def test_plain_pink_basket_region_yields_nothing(self):
        frame = background()
        cv2.rectangle(frame, (100, 60), (540, 240), BASKET_BGR[0], -1)
        cv2.rectangle(frame, (100, 240), (540, 420), BASKET_BGR[1], -1)
        self.assertEqual(detect_eggs(frame), ())

    def test_egg_cut_by_the_frame_border_is_found(self):
        detections = detect_eggs(draw_egg(background(), (70, 240), (130, 100), "green"))
        self.assertEqual(len(detections), 1)
        det = detections[0]
        self.assertEqual((det.color, det.touches_border), ("green", True))
        self.assertLessEqual(abs(det.w * WIDTH - 201), TOLERANCE_PX)  # visible part: x 0..200

    def test_ellipse_fill_applies_only_away_from_the_border(self):
        strict = DetectorParams(ellipse_fill_range=(2.0, 3.0))  # no real outline can pass
        self.assertEqual(detect_eggs(draw_egg(background(), (320, 240), (130, 100), "green"), strict), ())
        cut = draw_egg(background(), (70, 240), (130, 100), "green")
        self.assertEqual([det.touches_border for det in detect_eggs(cut, strict)], [True])

    def test_gap_repair_accepts_a_notched_egg(self):
        frame = draw_egg(background(), (320, 230), (130, 100), "green")
        for dx in (-70, 0, 70):  # dark notches, like tarp reflections cutting the white
            cv2.line(frame, (320 + dx, 265), (320 + int(dx * 1.2), 345), BROWN, 32)
        without = inspect_candidates(frame, DetectorParams(repair_kernel_fraction=0.0))[0]
        self.assertEqual(without.rejected, "solidity")
        self.assertEqual([det.color for det in detect_eggs(frame)], ["green"])

    def test_candidates_report_rejection_reasons(self):
        frame = draw_egg(background(), (130, 130), (100, 78), "green")
        cv2.ellipse(frame, (470, 150), (120, 110), 0, 0, 360, WHITE, -1)  # huge white shape, one small spot
        cv2.circle(frame, (470, 150), 18, SPOT_BGR["green"], -1)
        cv2.rectangle(frame, (60, 330), (620, 360), WHITE, -1)  # long bar with a spot
        cv2.circle(frame, (340, 345), 15, SPOT_BGR["green"], -1)
        cv2.circle(frame, (120, 440), 20, SPOT_BGR["green"], -1)  # spot alone
        cv2.circle(frame, (560, 440), 4, SPOT_BGR["green"], -1)  # speck
        # the window (spots +- 2 d) clips the big white shape, so its sparse spot is what fails; with
        # the edge stage the lone spot (no white ring) and the speck (too small) are no spots at all
        reasons = sorted(str(candidate.rejected) for candidate in inspect_candidates(frame))
        self.assertEqual(reasons, ["None", "aspect", "spot fraction"])
        hsv = sorted(str(candidate.rejected) for candidate in inspect_candidates(frame, HSV))
        self.assertEqual(hsv, ["None", "area", "aspect", "spot fraction", "spot size"])

    def test_scale_sanity_rejects_a_spot_too_big_for_its_egg(self):
        frame = background()
        cv2.ellipse(frame, (320, 240), (45, 35), 0, 0, 360, WHITE, -1)
        cv2.circle(frame, (320, 240), 30, SPOT_BGR["green"], -1)  # spot 60 px on a 70 px egg: scale 1.2
        (candidate,) = inspect_candidates(frame)
        self.assertEqual(candidate.rejected, "scale")
        self.assertLess(candidate.scale, 2.5)
        self.assertEqual(detect_eggs(frame), ())

    def test_blue_spotted_egg_is_not_reported_as_another_color(self):
        frame = np.full((HEIGHT, WIDTH, 3), TARP_BGR, dtype=np.uint8)  # blue eggs were dropped 2026-09-24
        center, axes = (320, 250), (130, 100)
        cv2.ellipse(frame, center, axes, 0, 0, 360, WHITE, -1)
        for fx, fy in SPOT_LAYOUT:
            cv2.circle(frame, (center[0] + int(fx * axes[0]), center[1] + int(fy * axes[1])), axes[1] // 4,
                       BLUE_SPOT_BGR, -1)
        self.assertEqual(detect_eggs(frame), ())
        self.assertEqual(detect_eggs(frame, HSV), ())

    def test_blue_tarp_patch_next_to_a_white_glint_is_nothing(self):
        frame = np.full((HEIGHT, WIDTH, 3), (110, 60, 30), dtype=np.uint8)  # lighter tarp
        cv2.circle(frame, (300, 240), 40, TARP_BGR, -1)  # darker, spot-sized tarp patch
        cv2.ellipse(frame, (380, 240), (35, 60), 0, 0, 360, WHITE, -1)  # glint beside it, not around it
        self.assertEqual(detect_eggs(frame), ())

    def test_ring_check_rejects_a_spot_without_white_around_it(self):
        frame = background()
        cv2.circle(frame, (320, 240), 40, SPOT_BGR["green"], -1)  # green disk on tarp: no white ring
        cv2.ellipse(frame, (320, 380), (120, 60), 0, 0, 360, WHITE, -1)  # white shape nearby
        self.assertEqual(inspect_candidates(frame), ())
        self.assertEqual(detect_eggs(frame), ())

    def test_edge_spot_stage_finds_the_same_egg(self):
        center, axes = (330, 250), (130, 100)
        frame = draw_egg(background(), center, axes, "green")
        detections = detect_eggs(frame, DetectorParams(spot_detector="edge"))
        self.assertEqual([det.color for det in detections], ["green"])
        self.assert_box(detections[0], center, axes)
        with self.assertRaises(ValueError):
            detect_eggs(frame, DetectorParams(spot_detector="sift"))

    def test_edge_stage_falls_back_to_hsv_without_contrib(self):
        from types import SimpleNamespace
        from unittest import mock

        from robot.vision import egg_detector
        from robot.vision.spot_edges import edge_available, edge_spot_blobs

        self.assertFalse(edge_available(SimpleNamespace()))
        self.assertIsNone(edge_spot_blobs(SimpleNamespace(), None, None, DetectorParams()))
        frame = draw_egg(background(), (330, 250), (130, 100), "green")
        with mock.patch.object(egg_detector, "edge_spot_blobs", return_value=None):
            self.assertEqual([det.color for det in detect_eggs(frame, DetectorParams(spot_detector="edge"))],
                             ["green"])

    def test_rejects_non_color_frames(self):
        with self.assertRaises(ValueError):
            detect_eggs(np.zeros((HEIGHT, WIDTH), dtype=np.uint8))


if __name__ == "__main__":
    unittest.main()
