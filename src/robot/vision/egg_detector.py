"""src/robot/vision/egg_detector.py: Color and shape egg detector for the front camera (Auto Catch).

Eggs are white ellipsoids with green, blue, or red spots (owner decision, 2026-09-24). Pipeline
per BGR frame: HSV -> (white-body mask OR spot masks) AND NOT the pink basket -> morphological
open (drops thin clutter) and close -> outer contours -> keep contours with enough area, an
egg-like bbox aspect, a convex outline (solidity), and, when clear of the frame border, an
elliptical fill; then count the spot blobs of each color inside the filled contour. At least
EGG_MIN_SPOTS are required and the color with the most spot pixels names the egg. The basket is
removed from every mask, so it can neither join an egg nor add spots (robot bug, 2026-09-24).
Spots stay in the component mask because large, dark spots would otherwise split the white body.
Thresholds are placeholders in config.py. OpenCV is imported lazily (see __init__).
"""

from collections.abc import Mapping
from dataclasses import dataclass, field
import math
from typing import Any

import numpy as np

from robot.config import (
    BASKET_HSV,
    EGG_ASPECT_RANGE,
    EGG_BODY_HSV,
    EGG_BORDER_MARGIN_PX,
    EGG_CLOSE_KERNEL_PX,
    EGG_ELLIPSE_FILL_RANGE,
    EGG_MIN_AREA_PX,
    EGG_MIN_SOLIDITY,
    EGG_MIN_SPOT_AREA_PX,
    EGG_MIN_SPOTS,
    EGG_OPEN_KERNEL_PX,
    EGG_SPOT_HSV,
)
from robot.vision.egg_size import EggDetection

__all__ = ("DetectorParams", "EggDetection", "best_egg", "detect_eggs")

HsvRange = tuple[tuple[int, int, int], tuple[int, int, int]]
ELLIPSE_MIN_POINTS = 5  # cv2.fitEllipse needs at least five contour points


@dataclass(frozen=True)
class DetectorParams:
    """All detector thresholds; the defaults come from config.py."""

    body_hsv: HsvRange = EGG_BODY_HSV
    spot_hsv: Mapping[str, tuple[HsvRange, ...]] = field(default_factory=lambda: EGG_SPOT_HSV)
    basket_hsv: HsvRange = BASKET_HSV
    min_area_px: int = EGG_MIN_AREA_PX
    aspect_range: tuple[float, float] = EGG_ASPECT_RANGE
    min_spots: int = EGG_MIN_SPOTS
    open_kernel_px: int = EGG_OPEN_KERNEL_PX
    close_kernel_px: int = EGG_CLOSE_KERNEL_PX
    min_spot_area_px: int = EGG_MIN_SPOT_AREA_PX
    min_solidity: float = EGG_MIN_SOLIDITY
    ellipse_fill_range: tuple[float, float] = EGG_ELLIPSE_FILL_RANGE
    border_margin_px: int = EGG_BORDER_MARGIN_PX


DEFAULT_PARAMS = DetectorParams()


@dataclass(frozen=True)
class _Masks:
    """Basket-free masks of one frame: the component mask and one spot mask per color."""

    components: Any
    spots: Mapping[str, Any]


def _cv2() -> Any:
    import cv2  # lazy: see the module docstring

    return cv2


def _in_ranges(cv2: Any, hsv: Any, ranges: tuple[HsvRange, ...]) -> Any:
    masks = [cv2.inRange(hsv, np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8)) for low, high in ranges]
    return np.bitwise_or.reduce(masks) if masks else np.zeros(hsv.shape[:2], dtype=np.uint8)


def _masks(cv2: Any, frame: Any, params: DetectorParams) -> _Masks:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    not_basket = cv2.bitwise_not(_in_ranges(cv2, hsv, (params.basket_hsv,)))
    spots = {color: _in_ranges(cv2, hsv, ranges) & not_basket for color, ranges in params.spot_hsv.items()}
    union = np.bitwise_or.reduce([_in_ranges(cv2, hsv, (params.body_hsv,)) & not_basket, *spots.values()])
    opened = union
    if params.open_kernel_px > 1:
        open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (params.open_kernel_px,) * 2)
        opened = cv2.morphologyEx(union, cv2.MORPH_OPEN, open_kernel)
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (params.close_kernel_px,) * 2)
    return _Masks(components=cv2.morphologyEx(opened, cv2.MORPH_CLOSE, close_kernel), spots=spots)


def _spot_blobs(cv2: Any, mask: Any, min_area: int) -> tuple[int, int]:
    """(number of blobs with area >= min_area, their total pixels) in a 0/1 mask."""
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    kept = [int(stats[index, cv2.CC_STAT_AREA]) for index in range(1, count)
            if stats[index, cv2.CC_STAT_AREA] >= min_area]
    return len(kept), sum(kept)


def _spots(cv2: Any, inside: Any, crops: Mapping[str, Any], params: DetectorParams) -> tuple[str, int] | None:
    """(color, total spot blobs) inside the filled contour, or None when there are too few spots."""
    blobs = {color: _spot_blobs(cv2, ((mask > 0) & inside).astype(np.uint8), params.min_spot_area_px)
             for color, mask in crops.items()}
    total = sum(count for count, _ in blobs.values())
    if total < params.min_spots:
        return None
    return max(blobs, key=lambda name: blobs[name][1]), total


def _shape_ok(cv2: Any, contour: Any, area: float, touches_border: bool, params: DetectorParams) -> float | None:
    """The solidity when the outline is egg-shaped, else None.

    Convex (solidity) always; elliptical fill only when the whole outline is visible.
    """
    hull_area = cv2.contourArea(cv2.convexHull(contour))
    solidity = area / hull_area if hull_area > 0 else 0.0
    if solidity < params.min_solidity:
        return None
    if touches_border:
        return solidity
    if len(contour) < ELLIPSE_MIN_POINTS:
        return None
    _, (axis_a, axis_b), _ = cv2.fitEllipse(contour)
    ellipse_area = math.pi * axis_a * axis_b / 4
    low, high = params.ellipse_fill_range
    return solidity if ellipse_area > 0 and low <= area / ellipse_area <= high else None


def _contour_egg(cv2: Any, contour: Any, masks: _Masks, params: DetectorParams) -> EggDetection | None:
    """The egg for one outer contour, or None when area, aspect, shape, or spots do not fit."""
    area = cv2.contourArea(contour)
    x, y, w, h = cv2.boundingRect(contour)
    low_aspect, high_aspect = params.aspect_range
    if area < params.min_area_px or not low_aspect <= w / h <= high_aspect:
        return None
    height, width = masks.components.shape
    margin = params.border_margin_px
    touches_border = x < margin or y < margin or x + w > width - margin or y + h > height - margin
    solidity = _shape_ok(cv2, contour, area, touches_border, params)
    if solidity is None:
        return None
    inside = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(inside, [contour], -1, 1, thickness=-1, offset=(-x, -y))
    spots = _spots(cv2, inside > 0, {color: mask[y:y + h, x:x + w] for color, mask in masks.spots.items()}, params)
    if spots is None:
        return None
    return EggDetection(cx=(x + w / 2) / width, cy=(y + h / 2) / height, w=w / width, h=h / height,
                        color=spots[0], spots=spots[1], area_px=int(area), touches_border=touches_border,
                        solidity=solidity)


def detect_eggs(frame_bgr: Any, params: DetectorParams = DEFAULT_PARAMS) -> tuple[EggDetection, ...]:
    """All eggs in an HxWx3 uint8 BGR frame, largest (by contour area) first."""
    cv2 = _cv2()
    frame = np.asarray(frame_bgr, dtype=np.uint8)
    if frame.ndim != 3 or frame.shape[2] != 3 or 0 in frame.shape:
        raise ValueError(f"Expected an HxWx3 BGR frame, got shape {frame.shape}")
    masks = _masks(cv2, frame, params)
    contours, _ = cv2.findContours(masks.components, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    eggs = (_contour_egg(cv2, contour, masks, params) for contour in contours)
    return tuple(sorted((egg for egg in eggs if egg is not None), key=lambda det: det.area_px, reverse=True))


def best_egg(frame_bgr: Any, params: DetectorParams = DEFAULT_PARAMS) -> EggDetection | None:
    """The largest detection, or None."""
    detections = detect_eggs(frame_bgr, params)
    return detections[0] if detections else None
