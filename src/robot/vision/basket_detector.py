"""src/robot/vision/basket_detector.py: Pink basket detector for the front camera (Auto Release).

Thresholds the frame with BASKET_DETECT_HSV (looser than the egg detector's BASKET_HSV exclusion:
at the release position the basket is dark and desaturated), opens (BASKET_OPEN_PX, removes red egg
spots and specks) and closes (BASKET_CLOSE_PX, joins the basket's weave and handle gaps), and keeps
the largest connected component when it covers at least BASKET_MIN_AREA_FRACTION of the frame and
fills at least BASKET_MIN_FILL of its bbox. Kernel sizes are full-resolution pixels; the morphology
runs on a mask downscaled by BASKET_MORPH_SCALE to stay within a few milliseconds per frame.
OpenCV is imported lazily (see robot/vision/__init__.py).
"""

from dataclasses import dataclass
from typing import Any

import numpy as np

from robot.vision.basket_size import BasketDetection
from robot.vision.config_vision import (
    BASKET_CLOSE_PX,
    BASKET_DETECT_HSV,
    BASKET_MIN_AREA_FRACTION,
    BASKET_MIN_FILL,
    BASKET_MORPH_SCALE,
    BASKET_OPEN_PX,
)

HsvRange = tuple[tuple[int, int, int], tuple[int, int, int]]


@dataclass(frozen=True)
class BasketParams:
    """Basket detector thresholds; the defaults come from config_vision.py."""

    hsv: HsvRange = BASKET_DETECT_HSV
    open_px: int = BASKET_OPEN_PX
    close_px: int = BASKET_CLOSE_PX
    morph_scale: int = BASKET_MORPH_SCALE
    min_area_fraction: float = BASKET_MIN_AREA_FRACTION
    min_fill: float = BASKET_MIN_FILL


DEFAULT_BASKET_PARAMS = BasketParams()


def _odd(value: float) -> int:
    return max(3, int(round(value))) | 1


def _kernel(cv2: Any, size_px: int, scale: int) -> Any:
    side = _odd(size_px / scale)
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (side, side))


def basket_mask(frame_bgr: Any, params: BasketParams = DEFAULT_BASKET_PARAMS) -> Any:
    """The opened and closed pink mask (0/255) at 1 / morph_scale of the frame size."""
    import cv2  # lazy: see the module docstring

    frame = np.asarray(frame_bgr, dtype=np.uint8)
    if frame.ndim != 3 or frame.shape[2] != 3 or 0 in frame.shape:
        raise ValueError(f"Expected an HxWx3 BGR frame, got shape {frame.shape}")
    low, high = params.hsv
    mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), np.array(low, dtype=np.uint8),
                       np.array(high, dtype=np.uint8))
    scale = max(1, int(params.morph_scale))
    if scale > 1:
        height, width = mask.shape
        small = cv2.resize(mask, (max(1, width // scale), max(1, height // scale)), interpolation=cv2.INTER_AREA)
        mask = np.where(small >= 128, 255, 0).astype(np.uint8)  # majority vote per scale x scale block
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _kernel(cv2, params.open_px, scale))
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _kernel(cv2, params.close_px, scale))


def largest_component(mask: Any) -> BasketDetection | None:
    """The largest component of a 0/255 mask as a normalized detection (no thresholds), or None."""
    import cv2  # lazy: see the module docstring

    count, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count < 2:
        return None
    index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    x, y, w, h, area = (int(value) for value in stats[index, :5])
    height, width = mask.shape[:2]
    return BasketDetection(cx=(x + w / 2) / width, cy=(y + h / 2) / height, w=w / width, h=h / height,
                           area_fraction=area / (width * height), fill=area / (w * h))


def detect_basket(frame_bgr: Any, params: BasketParams = DEFAULT_BASKET_PARAMS) -> BasketDetection | None:
    """The pink basket in an HxWx3 uint8 BGR frame, or None when the largest pink component is
    smaller than min_area_fraction of the frame or fills less than min_fill of its bbox."""
    found = largest_component(basket_mask(frame_bgr, params))
    if found is None or found.area_fraction < params.min_area_fraction or found.fill < params.min_fill:
        return None
    return found
