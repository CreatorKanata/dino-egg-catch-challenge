"""src/robot/vision/basket_size.py: The basket detection record and the Auto Release size precondition.

BasketDetection is plain data (normalized bbox of the largest pink component, its share of the
frame, and how much of its bbox it fills), produced by basket_detector.py and read by the Auto
Release state machine, the mode manager, and the signboard overlays. classify_basket() is the
precondition checked at the Thx press. The basket never counts as too close: at the release
position it fills most of the frame by design. Stdlib-only, so the pure decision modules and the
signboard child can import it without OpenCV.
"""

from dataclasses import dataclass
from typing import Final, Literal, Protocol

from robot.vision.config_vision import AUTO_RELEASE_MIN_BASKET_W

BasketSizeClass = Literal["none", "too_small", "ok"]
BASKET_SIZE_CLASSES: Final = ("none", "too_small", "ok")


@dataclass(frozen=True)
class BasketDetection:
    """The pink basket: bbox center and size normalized to the frame (0..1), component area as a
    fraction of the frame, and component area / bbox area."""

    cx: float
    cy: float
    w: float
    h: float
    area_fraction: float
    fill: float


class HasWidth(Protocol):
    """Anything with a normalized bbox width (BasketDetection or a test double)."""

    w: float


def classify_basket(det: HasWidth | None, min_w: float = AUTO_RELEASE_MIN_BASKET_W) -> BasketSizeClass:
    """"none" without a basket, "too_small" (too far) below min_w, otherwise "ok"; min_w itself is ok."""
    if det is None:
        return "none"
    return "too_small" if det.w < min_w else "ok"
