"""src/robot/signboard_layout.py: Pure layout and status text for the attendee signboard.

Computes where the three camera views and the status bar go and what the status bar says,
using only the stdlib (own Rect, not pygame.Rect) so it is unit-tested without a display.
signboard.py does the drawing.
"""

from dataclasses import dataclass

from robot.config import SignboardTheme
from robot.dino_controller_reader import ControllerState
from robot.drive_state import DriveState


@dataclass(frozen=True)
class Rect:
    """Axis-aligned rectangle in window pixels."""

    x: int
    y: int
    w: int
    h: int

    def fit(self, src_w: int, src_h: int) -> "Rect":
        """Largest rect with the source aspect ratio, centered inside this one (letterbox)."""
        if src_w <= 0 or src_h <= 0:
            raise ValueError(f"Source size must be positive, got {src_w}x{src_h}")
        scale = min(self.w / src_w, self.h / src_h)
        w = min(self.w, max(1, round(src_w * scale)))
        h = min(self.h, max(1, round(src_h * scale)))
        return Rect(self.x + (self.w - w) // 2, self.y + (self.h - h) // 2, w, h)


@dataclass(frozen=True)
class Layout:
    """Main camera (left two thirds), two stacked side cameras (right third), status bar."""

    main: Rect
    sides: tuple[Rect, Rect]
    status: Rect


@dataclass(frozen=True)
class Glyphs:
    """Symbols for the speed footprints and held directions."""

    filled: str
    empty: str
    up: str
    down: str
    left: str
    right: str
    none: str
    rotate_left: str
    rotate_right: str


UNICODE_GLYPHS = Glyphs(filled="●", empty="○", up="↑", down="↓", left="←", right="→", none="–",
                        rotate_left="↺", rotate_right="↻")
ASCII_GLYPHS = Glyphs(filled="#", empty="-", up="UP", down="DOWN", left="LEFT", right="RIGHT", none="-",
                      rotate_left="CCW", rotate_right="CW")


def compute_layout(size: tuple[int, int], margin: int, status_height: int) -> Layout:
    """Split the window; raise ValueError when it is too small for the margins and status bar."""
    width, height = size
    content_h = height - status_height - 3 * margin
    content_w = width - 3 * margin
    side_h = (content_h - margin) // 2
    if content_w < 3 or side_h < 1 or status_height < 1:
        raise ValueError(f"Window {width}x{height} is too small for the signboard layout")
    main_w = content_w * 2 // 3
    side_x = 2 * margin + main_w
    side_w = content_w - main_w
    return Layout(
        main=Rect(margin, margin, main_w, content_h),
        sides=(Rect(side_x, margin, side_w, side_h), Rect(side_x, 2 * margin + side_h, side_w, side_h)),
        status=Rect(margin, height - margin - status_height, width - 2 * margin, status_height),
    )


def _mode_text(drive: DriveState) -> str:
    if drive.input_lost:
        return "INPUT LOST"
    if drive.catch_requested:
        return "CATCH!"
    return "DRIVE"


def _speed_text(drive: DriveState, level_count: int, glyphs: Glyphs) -> str:
    level = drive.speed_index + 1
    footprints = glyphs.filled * level + glyphs.empty * (level_count - level)
    return f"{footprints} speed {level}/{level_count}"


def _direction_text(controller: ControllerState, pending_rotation_deg: float, glyphs: Glyphs) -> str:
    """Held joystick directions, then the encoder rotation symbol while a rotation is pending."""
    held = (
        (controller.up, glyphs.up),
        (controller.down, glyphs.down),
        (controller.left, glyphs.left),
        (controller.right, glyphs.right),
        (pending_rotation_deg > 0, glyphs.rotate_left),
        (pending_rotation_deg < 0, glyphs.rotate_right),
    )
    return " ".join(symbol for active, symbol in held if active) or glyphs.none


def status_lines(
    drive: DriveState,
    controller: ControllerState,
    level_count: int,
    glyphs: Glyphs = UNICODE_GLYPHS,
) -> tuple[str, ...]:
    """Mode, speed footprints (level fixed by config), and held directions plus rotation."""
    return (
        _mode_text(drive),
        _speed_text(drive, level_count, glyphs),
        _direction_text(controller, drive.pending_rotation_deg, glyphs),
    )


def status_color(drive: DriveState, theme: SignboardTheme) -> tuple[int, int, int]:
    """Warning when input is lost, accent while Catch is requested, text color otherwise."""
    if drive.input_lost:
        return theme.warning
    if drive.catch_requested:
        return theme.accent
    return theme.text
