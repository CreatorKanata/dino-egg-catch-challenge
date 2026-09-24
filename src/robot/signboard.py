"""src/robot/signboard.py: Attendee-facing pygame signboard for Manual Mode.

Shows the overhead camera large, the Pi front and wrist cameras stacked on the right, and a
status bar (mode, notice or drive status, arm status, speed footprints, held directions). The
window keeps keyboard focus so KachiButton phrases arrive as text input; pump() returns that
text together with whether ESC or the window close button ended Manual Mode. Layout and text
live in signboard_layout.py; this module only draws and never blocks (the loop owns timing).
"""

from collections.abc import Mapping
from dataclasses import astuple, dataclass
import logging
import os
from typing import Any

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")  # keep stdout for startup guidance

import numpy as np  # noqa: E402
import pygame  # noqa: E402

from robot.config import (
    DEFAULT_THEME,
    SIGNBOARD_FONT_SIZE,
    SIGNBOARD_FRAME_WIDTH,
    SIGNBOARD_FULLSCREEN,
    SIGNBOARD_MAIN_CAMERA,
    SIGNBOARD_MARGIN,
    SIGNBOARD_SIDE_CAMERAS,
    SIGNBOARD_SIZE,
    SIGNBOARD_SMALL_FONT_SIZE,
    SIGNBOARD_STATUS_HEIGHT,
    SIGNBOARD_TITLE,
    SPEED_LEVELS,
    SignboardTheme,
)
from robot.dino_controller_reader import ControllerState
from robot.display_status import DisplayStatus
from robot.drive_state import DriveState
from robot.signboard_layout import (
    ASCII_GLYPHS,
    UNICODE_GLYPHS,
    Glyphs,
    Rect,
    compute_layout,
    status_color,
    status_lines,
)

logger = logging.getLogger(__name__)

# A private-use code point: the default font draws its "missing glyph" box for it.
MISSING_GLYPH_PROBE = ""
LABEL_PADDING = 6


@dataclass(frozen=True)
class PumpResult:
    """Outcome of one event pump: False after ESC or window close, plus text typed meanwhile."""

    keep_running: bool
    typed: str = ""


def _to_pygame_rect(rect: Rect) -> pygame.Rect:
    return pygame.Rect(rect.x, rect.y, rect.w, rect.h)


def pick_glyphs(font: Any) -> Glyphs:
    """Unicode glyphs when the font really has them, else ASCII (avoids tofu boxes)."""
    missing = font.metrics(MISSING_GLYPH_PROBE)
    symbols = "".join(astuple(UNICODE_GLYPHS))  # every symbol the status bar may draw
    supported = all(
        metrics is not None and [metrics] != missing for metrics in font.metrics(symbols)
    )
    return UNICODE_GLYPHS if supported else ASCII_GLYPHS


def _is_exit(event: Any) -> bool:
    return event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE)


class SignboardView:
    """pygame window: open(), render() once per loop frame, pump() for exit and text, close()."""

    def __init__(
        self,
        size: tuple[int, int] = SIGNBOARD_SIZE,
        fullscreen: bool = SIGNBOARD_FULLSCREEN,
        theme: SignboardTheme = DEFAULT_THEME,
        title: str = SIGNBOARD_TITLE,
    ) -> None:
        self._size = size
        self._fullscreen = fullscreen
        self._theme = theme
        self._title = title
        self._layout = compute_layout(size, SIGNBOARD_MARGIN, SIGNBOARD_STATUS_HEIGHT)
        self._screen: Any = None
        self._font: Any = None
        self._small_font: Any = None
        self._glyphs = ASCII_GLYPHS

    @property
    def surface(self) -> Any:
        return self._require_open()

    def open(self) -> None:
        pygame.init()
        pygame.display.set_caption(self._title)
        flags = pygame.FULLSCREEN if self._fullscreen else 0
        self._screen = pygame.display.set_mode(self._size, flags)
        self._font = pygame.font.Font(None, SIGNBOARD_FONT_SIZE)
        self._small_font = pygame.font.Font(None, SIGNBOARD_SMALL_FONT_SIZE)
        self._glyphs = pick_glyphs(self._font)
        pygame.key.start_text_input()  # KachiButton phrases arrive as TEXTINPUT events
        logger.info("Signboard opened (%dx%d, fullscreen=%s)", *self._size, self._fullscreen)

    def render(
        self,
        frames: Mapping[str, Any],
        drive: DriveState,
        controller: ControllerState,
        status: DisplayStatus = DisplayStatus(),
    ) -> None:
        """Draw one frame; frames are HWC uint8 BGR arrays or None (no signal)."""
        screen = self._require_open()
        screen.fill(self._theme.background)
        cameras = ((SIGNBOARD_MAIN_CAMERA, self._layout.main),
                   *zip(SIGNBOARD_SIDE_CAMERAS, self._layout.sides))
        for name, rect in cameras:
            self._draw_camera(screen, name, frames.get(name), rect)
        self._draw_status(screen, drive, controller, status)
        pygame.display.flip()

    def pump(self) -> PumpResult:
        """Process pending events: stop after ESC or window close; collect typed text in order."""
        self._require_open()
        events = pygame.event.get()
        typed = "".join(event.text for event in events if event.type == pygame.TEXTINPUT)
        return PumpResult(keep_running=not any(_is_exit(event) for event in events), typed=typed)

    def close(self) -> None:
        if self._screen is None:
            return
        self._screen = None
        pygame.display.quit()
        pygame.quit()
        logger.info("Signboard closed")

    def _draw_camera(self, screen: Any, name: str, frame: Any, rect: Rect) -> None:
        if frame is None:
            self._draw_no_signal(screen, name, rect)
        else:
            height, width = frame.shape[:2]
            image = pygame.image.frombuffer(np.ascontiguousarray(frame).tobytes(), (width, height), "BGR")
            target = rect.fit(width, height)
            screen.blit(pygame.transform.smoothscale(image, (target.w, target.h)), (target.x, target.y))
        pygame.draw.rect(screen, self._theme.frame, _to_pygame_rect(rect), SIGNBOARD_FRAME_WIDTH)
        label = self._small_font.render(name, True, self._theme.text)
        screen.blit(label, (rect.x + SIGNBOARD_FRAME_WIDTH + LABEL_PADDING,
                            rect.y + SIGNBOARD_FRAME_WIDTH + LABEL_PADDING))

    def _draw_no_signal(self, screen: Any, name: str, rect: Rect) -> None:
        text = self._small_font.render(f"{name}: no signal", True, self._theme.text)
        screen.blit(text, text.get_rect(center=_to_pygame_rect(rect).center))

    def _draw_status(self, screen: Any, drive: DriveState, controller: ControllerState, status: DisplayStatus) -> None:
        box = _to_pygame_rect(self._layout.status)
        pygame.draw.rect(screen, self._theme.frame, box, SIGNBOARD_FRAME_WIDTH)
        color = status_color(drive, self._theme, status)
        lines = status_lines(drive, controller, len(SPEED_LEVELS), self._glyphs, status)
        fonts = (self._font,) + (self._small_font,) * (len(lines) - 1)
        surfaces = [font.render(line, True, color) for font, line in zip(fonts, lines)]
        top = box.centery - sum(surface.get_height() for surface in surfaces) // 2
        for surface in surfaces:
            screen.blit(surface, surface.get_rect(midtop=(box.centerx, top)))
            top += surface.get_height()

    def _require_open(self) -> Any:
        if self._screen is None:
            raise RuntimeError("Signboard is not open; call open() first")
        return self._screen
