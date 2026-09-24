"""tests/robot/test_signboard.py: Headless smoke tests of the pygame signboard window.

Uses SDL's dummy video driver so rendering and event handling run without a display. The
tests check that drawing does not raise and that ESC ends the view; pixels are not asserted.
"""

import os

os.environ["SDL_VIDEODRIVER"] = "dummy"
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

from dataclasses import replace  # noqa: E402
import unittest  # noqa: E402

try:
    import numpy as np
    import pygame
except ImportError:  # pragma: no cover - depends on the environment
    pygame = None

from robot.config import TOP_CAMERA_KEY  # noqa: E402
from robot.dino_controller_reader import INITIAL_STATE  # noqa: E402
from robot.drive_state import DriveState  # noqa: E402

SIZE = (640, 400)
NO_FRAMES = {TOP_CAMERA_KEY: None, "front": None, "wrist": None}


class SignboardViewTests(unittest.TestCase):
    def setUp(self):
        if pygame is None:
            self.skipTest("pygame is not installed")
        from robot.signboard import SignboardView

        self.view = SignboardView(size=SIZE, fullscreen=False)
        self.view.open()
        self.addCleanup(self.view.close)

    def test_render_without_frames(self):
        self.view.render(NO_FRAMES, DriveState(), INITIAL_STATE)
        self.assertEqual(self.view.surface.get_size(), SIZE)

    def test_render_bgr_frame_and_status(self):
        frame = np.zeros((48, 64, 3), dtype=np.uint8)
        frame[..., 0] = 255  # blue channel in BGR order
        frames = {**NO_FRAMES, TOP_CAMERA_KEY: frame, "front": frame[:, ::-1]}
        controller = replace(INITIAL_STATE, synchronized=True, up=True, left=True)
        drive = DriveState(speed_index=1, catch_requested=True, input_lost=False)
        self.view.render(frames, drive, controller)
        self.assertEqual(self.view.surface.get_size(), SIZE)

    def test_pump_keeps_running_without_exit_events(self):
        self.assertTrue(self.view.pump())

    def test_escape_ends_the_view(self):
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_ESCAPE))
        self.assertFalse(self.view.pump())

    def test_window_close_ends_the_view(self):
        pygame.event.post(pygame.event.Event(pygame.QUIT))
        self.assertFalse(self.view.pump())

    def test_glyph_choice_detects_missing_glyph_boxes(self):
        from robot.signboard import pick_glyphs
        from robot.signboard_layout import ASCII_GLYPHS, UNICODE_GLYPHS

        class FakeFont:
            def __init__(self, box_for_symbols):
                self.box_for_symbols = box_for_symbols

            def metrics(self, text):
                box, real = (1, 9, 0, 16, 10), (0, 17, 0, 17, 17)
                return [box if (self.box_for_symbols or ch == "\ue000") else real for ch in text]

        self.assertIs(pick_glyphs(FakeFont(box_for_symbols=True)), ASCII_GLYPHS)
        self.assertIs(pick_glyphs(FakeFont(box_for_symbols=False)), UNICODE_GLYPHS)

    def test_render_before_open_raises(self):
        from robot.signboard import SignboardView

        with self.assertRaises(RuntimeError):
            SignboardView(size=SIZE).render(NO_FRAMES, DriveState(), INITIAL_STATE)


if __name__ == "__main__":
    unittest.main()
