"""tests/robot/test_signboard.py: Headless smoke tests of the pygame signboard window.

Uses SDL's dummy video driver so rendering and event handling run without a display. The
tests check that drawing (including the overlays) does not raise, that ESC ends the view, that
typed text (KachiButton) and the capture key are returned by pump(), and that serve() turns them
into command lines; pixels are asserted only for the egg outline color.
"""

import os

os.environ["SDL_VIDEODRIVER"] = "dummy"
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

from dataclasses import replace  # noqa: E402
import io  # noqa: E402
import unittest  # noqa: E402

try:
    import numpy as np
    import pygame
except ImportError:  # pragma: no cover - depends on the environment
    pygame = None

from robot.config import DEFAULT_THEME, TOP_CAMERA_KEY  # noqa: E402
from robot.dino_controller_reader import INITIAL_STATE  # noqa: E402
from robot.display_status import DisplayStatus, Overlay  # noqa: E402
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
        status = DisplayStatus(mode="fsc", voice_listening=True, notice="Listening...", arm_status="syncing")
        self.view.render(frames, drive, controller, status)
        self.assertEqual(self.view.surface.get_size(), SIZE)

    def test_render_overlays_on_front_view(self):
        frame = np.zeros((48, 64, 3), dtype=np.uint8)
        egg = Overlay("front", 0.5, 0.5, 0.5, 0.5, "egg_ok", "green egg")
        overlays = (Overlay("front", 0.56, 0.57, 0.64, 0.69, "target", "place the egg here"), egg,
                    Overlay("front", 0.2, 0.2, 0.1, 0.1, "egg_out", "red egg: too far"),
                    Overlay("wrist", 0.5, 0.5, 0.5, 0.5, "egg_ok", ""))  # wrist has no signal: skipped
        self.view.render({**NO_FRAMES, "front": frame}, DriveState(), INITIAL_STATE, DisplayStatus(overlays=overlays))
        from robot.signboard_layout import compute_layout, overlay_rect

        fit = compute_layout(SIZE, 16, 16 / 9, 96).sides[0].fit(64, 48)
        box = overlay_rect(egg, fit)
        left_edge = (box.x + 1, box.y + box.h // 2)  # on the ellipse outline, inside the 3 px line
        self.assertEqual(tuple(self.view.surface.get_at(left_edge))[:3], DEFAULT_THEME.ok)

    def test_target_label_below_and_egg_label_above(self):
        frame = np.zeros((48, 64, 3), dtype=np.uint8)
        target = Overlay("front", 0.5, 0.4, 0.5, 0.4, "target", "place the egg here")
        egg = Overlay("front", 0.5, 0.4, 0.5, 0.4, "egg_ok", "green egg")
        from robot.signboard_layout import compute_layout, overlay_rect

        fit = compute_layout(SIZE, 16, 16 / 9, 96).sides[0].fit(64, 48)
        box = overlay_rect(target, fit)
        for overlay, color, rows in ((target, DEFAULT_THEME.text, range(box.y + box.h + 1, fit.y + fit.h)),
                                     (egg, DEFAULT_THEME.ok, range(fit.y, box.y - 1))):
            with self.subTest(kind=overlay.kind):
                self.view.render({**NO_FRAMES, "front": frame}, DriveState(), INITIAL_STATE,
                                 DisplayStatus(overlays=(overlay,)))
                surface = self.view.surface
                hits = [(x, y) for y in rows for x in range(box.x, box.x + box.w)
                        if tuple(surface.get_at((x, y)))[:3] == color]
                self.assertTrue(hits, f"no {overlay.kind} label pixels on the expected side")

    def test_capture_key_is_reported(self):
        pygame.event.clear()
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_c))
        self.assertEqual(self.view.pump(), self._pump_result(True, "", capture=True))

    def test_pump_keeps_running_without_exit_events(self):
        pygame.event.clear()
        self.assertEqual(self.view.pump(), self._pump_result(True, ""))

    def test_pump_returns_typed_text_in_order(self):
        pygame.event.clear()
        for text in ("Go", " ", "Go!"):
            pygame.event.post(pygame.event.Event(pygame.TEXTINPUT, text=text))
        self.assertEqual(self.view.pump(), self._pump_result(True, "Go Go!"))

    def test_escape_ends_the_view(self):
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_ESCAPE))
        self.assertFalse(self.view.pump().keep_running)

    def test_window_close_ends_the_view(self):
        pygame.event.post(pygame.event.Event(pygame.QUIT))
        self.assertFalse(self.view.pump().keep_running)

    def test_serve_writes_one_line_per_command_and_stops_on_escape(self):
        from robot.signboard_process import serve

        pygame.event.clear()
        pygame.event.post(pygame.event.Event(pygame.TEXTINPUT, text="Stop"))
        pygame.event.post(pygame.event.Event(pygame.TEXTINPUT, text="Hi!"))
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_c))
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_ESCAPE))
        read_end, write_end = os.pipe()  # select() needs a real descriptor; nothing is ever written
        self.addCleanup(os.close, read_end)
        self.addCleanup(os.close, write_end)
        stdout = io.BytesIO()
        with os.fdopen(read_end, "rb", buffering=0, closefd=False) as stdin:
            self.assertEqual(serve(self.view, stdin, stdout, clock=lambda: 1.0), 0)
        self.assertEqual(stdout.getvalue(),
                         b'{"v": 1, "type": "command", "command": "stop"}\n'
                         b'{"v": 1, "type": "command", "command": "hi"}\n'
                         b'{"v": 1, "type": "command", "command": "capture"}\n')

    @staticmethod
    def _pump_result(keep_running, typed, capture=False):
        from robot.signboard import PumpResult

        return PumpResult(keep_running=keep_running, typed=typed, capture=capture)

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
