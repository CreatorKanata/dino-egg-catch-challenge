"""src/robot/vision/egg_size.py: The egg detection record and the Auto Catch size precondition.

EggDetection is plain data (normalized bbox, spot color), produced by egg_detector.py and read by
the alignment controller, the mode manager, and the signboard overlays. classify_size() is the
precondition checked at the Hi! press (docs/spec/operating-modes.md, section 4). Stdlib-only, so
the pure decision modules and the signboard child can import it without OpenCV.
"""

from dataclasses import dataclass
from typing import Final, Literal, Protocol

from robot.config import AUTO_CATCH_MAX_EGG_H, AUTO_CATCH_MIN_EGG_H

SizeClass = Literal["none", "too_small", "too_large", "ok"]
SIZE_CLASSES: Final = ("none", "too_small", "too_large", "ok")


@dataclass(frozen=True)
class EggDetection:
    """One egg: bbox center and size normalized to the frame (0..1), spot color, spot count, area."""

    cx: float
    cy: float
    w: float
    h: float
    color: str
    spots: int
    area_px: int


class HasHeight(Protocol):
    """Anything with a normalized bbox height (EggDetection or a test double)."""

    h: float


def classify_size(
    det: HasHeight | None,
    min_h: float = AUTO_CATCH_MIN_EGG_H,
    max_h: float = AUTO_CATCH_MAX_EGG_H,
) -> SizeClass:
    """"none" without an egg, "too_small" (too far) below min_h, "too_large" (too close) above
    max_h, otherwise "ok". The window bounds themselves count as ok."""
    if det is None:
        return "none"
    if det.h < min_h:
        return "too_small"
    if det.h > max_h:
        return "too_large"
    return "ok"
