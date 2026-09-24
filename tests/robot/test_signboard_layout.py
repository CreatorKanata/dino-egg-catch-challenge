"""tests/robot/test_signboard_layout.py: Hardware-free checks of the signboard layout and status text.

The layout and text helpers are pure, so geometry (letterboxing, no overlaps) and the status
wording for each Manual Mode and drive state are verified without pygame or a display.
"""

from dataclasses import replace
from itertools import combinations
import unittest

from robot.config import DEFAULT_THEME
from robot.dino_controller_reader import INITIAL_STATE
from robot.display_status import DisplayStatus, Overlay, front_overlays
from robot.vision.egg_size import EggDetection
from robot.drive_state import DriveState
from robot.signboard_layout import (
    ASCII_GLYPHS,
    UNICODE_GLYPHS,
    Rect,
    compute_layout,
    overlay_points,
    overlay_rect,
    status_color,
    status_lines,
)

SYNCED = replace(INITIAL_STATE, synchronized=True)
DRIVING = DriveState(speed_index=0, catch_requested=False, input_lost=False)


def overlaps(a, b):
    return a.x < b.x + b.w and b.x < a.x + a.w and a.y < b.y + b.h and b.y < a.y + a.h


def inside(rect, size):
    return rect.x >= 0 and rect.y >= 0 and rect.x + rect.w <= size[0] and rect.y + rect.h <= size[1]


class RectFitTests(unittest.TestCase):
    def test_wide_source_is_letterboxed_vertically(self):
        fitted = Rect(0, 0, 400, 400).fit(640, 320)
        self.assertEqual((fitted.w, fitted.h), (400, 200))
        self.assertEqual((fitted.x, fitted.y), (0, 100))

    def test_tall_source_is_pillarboxed_horizontally(self):
        fitted = Rect(10, 20, 400, 300).fit(480, 640)
        self.assertEqual((fitted.w, fitted.h), (225, 300))
        self.assertEqual((fitted.x, fitted.y), (10 + 87, 20))

    def test_same_aspect_fills_rect(self):
        self.assertEqual(Rect(5, 5, 320, 240).fit(640, 480), Rect(5, 5, 320, 240))

    def test_invalid_source_is_rejected(self):
        with self.assertRaises(ValueError):
            Rect(0, 0, 10, 10).fit(0, 10)


class OverlayRectTests(unittest.TestCase):
    def test_maps_normalized_box_into_letterboxed_rect(self):
        fit = Rect(0, 0, 400, 400).fit(640, 480)  # Rect(0, 50, 400, 300)
        box = overlay_rect(Overlay("front", 0.5, 0.5, 0.5, 0.5, "target", ""), fit)
        self.assertEqual(box, Rect(100, 125, 200, 150))

    def test_full_frame_and_offset_rect(self):
        fit = Rect(10, 20, 320, 240)
        self.assertEqual(overlay_rect(Overlay("front", 0.5, 0.5, 1.0, 1.0, "egg_ok", ""), fit), fit)
        corner = overlay_rect(Overlay("front", 0.1, 0.1, 0.2, 0.2, "egg_out", ""), fit)
        self.assertEqual(corner, Rect(10, 20, 64, 48))

    def test_overlay_points_unrotated_and_rotated(self):
        fit = Rect(0, 0, 400, 300)
        flat = overlay_points(Overlay("front", 0.5, 0.5, 0.5, 0.2, "target", ""), fit, count=4)
        self.assertEqual(flat, ((300, 150), (200, 180), (100, 150), (200, 120)))  # a = 100 px, b = 30 px
        turned = overlay_points(Overlay("front", 0.5, 0.5, 0.5, 0.2, "egg_ok", "", 90.0), fit, count=4)
        self.assertEqual(turned, ((200, 250), (170, 150), (200, 50), (230, 150)))  # first axis now vertical

    def test_tiny_box_keeps_one_pixel(self):
        box = overlay_rect(Overlay("front", 0.5, 0.5, 0.0, 0.0, "egg_ok", ""), Rect(0, 0, 100, 100))
        self.assertEqual((box.w, box.h), (1, 1))


class FrontOverlayTests(unittest.TestCase):
    def test_target_always_in_manual_and_nothing_in_fsc(self):
        from robot.mode_manager import AppState

        self.assertEqual([overlay.kind for overlay in front_overlays(AppState(), None)], ["target"])
        self.assertEqual(front_overlays(AppState(mode="fsc"), None), ())

    def test_basket_always_and_release_target_only_while_releasing(self):
        from robot.mode_manager import AppState
        from robot.vision.basket_size import BasketDetection

        basket = BasketDetection(0.52, 0.42, 0.78, 0.68, 0.33, 0.6)
        kinds = [overlay.kind for overlay in front_overlays(AppState(), None, basket)]
        self.assertEqual(kinds, ["target", "basket"])
        overlays = front_overlays(AppState(action="auto_release"), None, basket)
        self.assertEqual([overlay.kind for overlay in overlays], ["target", "release_target", "basket"])
        self.assertEqual((overlays[2].cx, overlays[2].w, overlays[2].label), (0.52, 0.78, "basket"))
        self.assertEqual(front_overlays(AppState(mode="fsc"), None, basket), ())

    def test_egg_uses_its_fitted_ellipse_else_its_bbox(self):
        from robot.mode_manager import AppState

        box_egg = EggDetection(0.5, 0.5, 0.4, 0.5, "green", 5, 9000)
        fitted = replace(box_egg, ellipse=(0.51, 0.49, 0.38, 0.6, 92.0))
        plain = front_overlays(AppState(), box_egg)[1]
        self.assertEqual((plain.cx, plain.w, plain.angle, plain.kind, plain.label),
                         (0.5, 0.4, 0.0, "egg_ok", "green egg"))
        rotated = front_overlays(AppState(), fitted)[1]
        self.assertEqual((rotated.cx, rotated.cy, rotated.w, rotated.h, rotated.angle), (0.51, 0.49, 0.38, 0.6, 92.0))
        far = front_overlays(AppState(), replace(box_egg, h=0.05))[1]
        self.assertEqual((far.kind, far.label), ("egg_out", "green egg: too far"))


class LayoutTests(unittest.TestCase):
    def test_rects_inside_window_and_disjoint(self):
        for size in ((1280, 720), (1920, 1080), (640, 480), (1280, 1024), (1600, 600)):
            with self.subTest(size=size):
                layout = compute_layout(size, 16, 16 / 9, 96)
                rects = (layout.main, *layout.sides, layout.status)
                self.assertTrue(all(inside(rect, size) for rect in rects))
                for a, b in combinations(rects, 2):
                    self.assertFalse(overlaps(a, b), (a, b))
                self.assertGreaterEqual(layout.status.h, 96)

    def test_main_fits_16_9_and_status_takes_the_rest(self):
        layout = compute_layout((1280, 720), 16, 16 / 9, 96)
        self.assertEqual((layout.main.w, layout.main.h), (821, 462))  # round(821 * 9 / 16)
        self.assertEqual(layout.main.fit(1280, 720), layout.main)  # a 16:9 frame fills it, no bars
        content = layout.main.w + layout.sides[0].w
        self.assertAlmostEqual(layout.main.w / content, 2 / 3, places=2)
        top, bottom = layout.sides
        self.assertEqual(top.x, bottom.x)
        self.assertLess(top.y + top.h, bottom.y)
        self.assertEqual(bottom.y + bottom.h, layout.main.y + layout.main.h)
        self.assertEqual(layout.status.y, layout.main.y + layout.main.h + 16)
        self.assertEqual(layout.status.y + layout.status.h, 720 - 16)
        self.assertEqual(layout.status.h, 720 - 3 * 16 - 462)  # 210 px, more than the 96 px minimum

    def test_short_window_caps_main_to_keep_the_status_minimum(self):
        layout = compute_layout((1600, 400), 16, 16 / 9, 96)
        self.assertEqual(layout.status.h, 96)
        self.assertEqual(layout.main.h, 400 - 3 * 16 - 96)

    def test_too_small_window_is_rejected(self):
        with self.assertRaises(ValueError):
            compute_layout((100, 100), 16, 16 / 9, 96)


class StatusTextTests(unittest.TestCase):
    def test_mode_line(self):
        self.assertEqual(status_lines(DRIVING, SYNCED, 3)[0], "MANUAL")
        self.assertEqual(status_lines(DRIVING, SYNCED, 3, status=DisplayStatus(mode="fsc"))[0], "FSC (demo)")
        busy = DisplayStatus(action="auto_catch")
        self.assertEqual(status_lines(DRIVING, SYNCED, 3, status=busy)[0], "MANUAL - AUTO CATCH")
        for phase, text in (("align", "aligning"), ("to_catch", "to catch pose"), ("wrist_check", "checking"),
                            ("pick_stub", "checking"), ("to_release", "to release pose")):
            with self.subTest(phase=phase):
                line = status_lines(DRIVING, SYNCED, 3, status=replace(busy, phase=phase))[0]
                self.assertEqual(line, f"MANUAL - AUTO CATCH: {text}")
        arm = status_lines(DRIVING, SYNCED, 3, status=DisplayStatus(action="auto_catch", arm_status="auto catch"))[2]
        self.assertTrue(arm.startswith("arm auto catch  |  "), arm)

    def test_drive_line_and_notice(self):
        lost = replace(DRIVING, input_lost=True, catch_requested=True)
        self.assertEqual(status_lines(lost, SYNCED, 3)[1], "INPUT LOST")
        self.assertEqual(status_lines(replace(DRIVING, catch_requested=True), SYNCED, 3)[1], "CATCH!")
        self.assertEqual(status_lines(DRIVING, SYNCED, 3)[1], "DRIVE")
        noticed = DisplayStatus(notice="Auto Catch: not available yet")
        self.assertEqual(status_lines(lost, SYNCED, 3, status=noticed)[1], "Auto Catch: not available yet")

    def test_detail_line_arm_status_and_speed(self):
        self.assertEqual(status_lines(DRIVING, SYNCED, 3)[2], "arm holding  |  ●○○ speed 1/3  |  –")
        fast = replace(DRIVING, speed_index=2)
        syncing = DisplayStatus(arm_status="syncing")
        self.assertEqual(status_lines(fast, SYNCED, 3, ASCII_GLYPHS, syncing)[2], "arm syncing  |  ### speed 3/3  |  -")
        for arm_status, text in (("following", "arm following"), ("leader fault", "LEADER ARM FAULT"),
                                 ("no leader", "no leader arm")):
            with self.subTest(arm_status=arm_status):
                line = status_lines(DRIVING, SYNCED, 3, status=DisplayStatus(arm_status=arm_status))[2]
                self.assertTrue(line.startswith(text + "  |  "), line)

    def test_voice_listening_is_shown(self):
        listening = DisplayStatus(mode="fsc", voice_listening=True)
        self.assertEqual(status_lines(DRIVING, SYNCED, 3, ASCII_GLYPHS, listening)[2],
                         "arm holding  |  mic on  |  #-- speed 1/3  |  -")

    def test_directions_with_both_glyph_sets(self):
        held = replace(SYNCED, up=True, right=True)
        self.assertTrue(status_lines(DRIVING, held, 3, UNICODE_GLYPHS)[2].endswith("  |  ↑ →"))
        self.assertTrue(status_lines(DRIVING, held, 3, ASCII_GLYPHS)[2].endswith("  |  UP RIGHT"))
        self.assertTrue(status_lines(DRIVING, SYNCED, 3, UNICODE_GLYPHS)[2].endswith("  |  –"))
        self.assertTrue(status_lines(DRIVING, SYNCED, 3, ASCII_GLYPHS)[2].endswith("  |  -"))

    def test_rotation_symbol_on_direction_line(self):
        left = replace(DRIVING, pending_rotation_deg=15.0)
        right = replace(DRIVING, pending_rotation_deg=-30.0)
        held = replace(SYNCED, up=True)
        cases = (
            (left, SYNCED, UNICODE_GLYPHS, "↺"),
            (right, held, UNICODE_GLYPHS, "↑ ↻"),
            (left, held, ASCII_GLYPHS, "UP CCW"),
            (right, SYNCED, ASCII_GLYPHS, "CW"),
            (DRIVING, held, UNICODE_GLYPHS, "↑"),
            (DRIVING, held, ASCII_GLYPHS, "UP"),
        )
        for drive, controller, glyphs, expected in cases:
            with self.subTest(expected=expected):
                self.assertEqual(status_lines(drive, controller, 3, glyphs)[2].split("  |  ")[-1], expected)

    def test_status_color(self):
        self.assertEqual(status_color(DriveState(input_lost=True, catch_requested=True), DEFAULT_THEME),
                         DEFAULT_THEME.warning)
        self.assertEqual(status_color(replace(DRIVING, catch_requested=True), DEFAULT_THEME), DEFAULT_THEME.accent)
        self.assertEqual(status_color(DRIVING, DEFAULT_THEME), DEFAULT_THEME.text)
        self.assertEqual(status_color(DRIVING, DEFAULT_THEME, DisplayStatus(stopped=True)), DEFAULT_THEME.warning)
        lost = DriveState(input_lost=True)
        self.assertEqual(status_color(lost, DEFAULT_THEME, DisplayStatus(mode="fsc")), DEFAULT_THEME.text)

    def test_warning_notice_uses_warning_color(self):
        alert = DisplayStatus(notice="Egg too far", notice_level="warning")
        self.assertEqual(status_color(DRIVING, DEFAULT_THEME, alert), DEFAULT_THEME.warning)
        self.assertEqual(status_color(DRIVING, DEFAULT_THEME, replace(alert, notice_level="info")), DEFAULT_THEME.text)
        expired = DisplayStatus(notice="", notice_level="warning")
        self.assertEqual(status_color(DRIVING, DEFAULT_THEME, expired), DEFAULT_THEME.text)

    def test_stopped_latch_lines(self):
        stopped = DisplayStatus(mode="fsc", notice="STOP", stopped=True)
        lines = status_lines(replace(DRIVING, input_lost=True), SYNCED, 3, status=stopped)
        self.assertEqual(lines[:2], ("STOPPED", "Press Go Go! to resume"))

    def test_auto_release_progress_recording_and_arm_text(self):
        releasing = DisplayStatus(action="auto_release", arm_status="auto release", progress=0.456)
        lines = status_lines(DRIVING, SYNCED, 3, ASCII_GLYPHS, releasing)
        self.assertEqual(lines[0], "MANUAL - AUTO RELEASE 46%")
        self.assertTrue(lines[2].startswith("arm auto release  |  "))
        recording = DisplayStatus(arm_status="following", recording_s=4.7)
        self.assertEqual(status_lines(DRIVING, SYNCED, 3, status=recording)[1], "REC release 4 s")
        self.assertEqual(status_color(DRIVING, DEFAULT_THEME, recording), DEFAULT_THEME.warning)
        noticed = replace(recording, notice="Recording release...")
        self.assertEqual(status_lines(DRIVING, SYNCED, 3, status=noticed)[1], "Recording release...")

    def test_fsc_second_line_is_the_voice_hint(self):
        lost = replace(DRIVING, input_lost=True)  # the controller is not used in FSC
        self.assertEqual(status_lines(lost, SYNCED, 3, status=DisplayStatus(mode="fsc"))[1], "Press Hi! to talk")
        listening = DisplayStatus(mode="fsc", voice_listening=True)
        self.assertEqual(status_lines(lost, SYNCED, 3, status=listening)[1], "Listening... (Hi! to end)")
        noticed = DisplayStatus(mode="fsc", notice="FSC")
        self.assertEqual(status_lines(lost, SYNCED, 3, status=noticed)[1], "FSC")


if __name__ == "__main__":
    unittest.main()
