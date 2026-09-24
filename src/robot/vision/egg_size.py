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
    """One egg: bbox center and size normalized to the frame (0..1), spot color, spot count, area,
    whether the bbox touches the frame border (partly visible egg), the outline's solidity, and
    its fitted ellipse. Alignment uses the bbox; the signboard draws the ellipse."""

    cx: float
    cy: float
    w: float
    h: float
    color: str
    spots: int
    area_px: int
    touches_border: bool = False
    solidity: float = 1.0
    # Fitted ellipse of the egg outline: center (normalized per axis), first and second axis
    # lengths divided by the frame width and height, and OpenCV's rotation of the first axis in
    # degrees. Dividing both axes by the matching frame side keeps it exact on any uniformly scaled
    # view (the signboard letterbox). None when no ellipse could be fitted.
    ellipse: tuple[float, float, float, float, float] | None = None


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
