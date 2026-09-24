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
from dataclasses import dataclass, field, replace
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
    EGG_REPAIR_KERNEL_FRACTION,
    EGG_SPOT_HSV,
)
from robot.vision.egg_size import EggDetection

__all__ = ("Candidate", "DetectorParams", "EggDetection", "best_egg", "detect_eggs", "inspect_candidates")

HsvRange = tuple[tuple[int, int, int], tuple[int, int, int]]
ELLIPSE_MIN_POINTS = 5  # cv2.fitEllipse needs at least five contour points
REPAIR_KERNEL_MAX_PX = 15  # performance bound for the gap repair (not a tunable)


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
    repair_kernel_fraction: float = EGG_REPAIR_KERNEL_FRACTION
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


def _spot_pixels(cv2: Any, inside: Any, crops: Mapping[str, Any], params: DetectorParams) -> dict[str, tuple[int, int]]:
    """Per color: (spot blobs >= min_spot_area_px, their pixels) inside the filled outline."""
    return {color: _spot_blobs(cv2, ((mask > 0) & inside).astype(np.uint8), params.min_spot_area_px)
            for color, mask in crops.items()}


@dataclass(frozen=True)
class Candidate:
    """One outer contour after open/close, with every measurement and the rule that rejected it
    (`rejected` is None for an egg). Pixel bbox; solidity and fill are measured after gap repair."""

    x: int
    y: int
    w: int
    h: int
    area_px: int
    aspect: float
    solidity: float
    fill: float | None
    touches_border: bool
    spots: Mapping[str, tuple[int, int]]
    rejected: str | None


def _repaired(cv2: Any, contour: Any, box: tuple[int, int, int, int], params: DetectorParams) -> tuple[Any, Any]:
    """(repaired outer contour in frame coordinates, its filled mask cropped to `box`).

    The outline is closed with a kernel proportional to the bbox (at least EGG_CLOSE_KERNEL_PX),
    so bites cut by glare or tarp reflections on the white do not fail the shape test. The close
    runs on a mask downscaled so the kernel stays near REPAIR_KERNEL_MAX_PX (a full-resolution
    60 px close cost ~30 ms per frame on a large egg).
    """
    x, y, w, h = box
    kernel_px = max(params.close_kernel_px, int(params.repair_kernel_fraction * min(w, h)))
    scale = max(1, math.ceil(kernel_px / REPAIR_KERNEL_MAX_PX))
    small_kernel = max(3, round(kernel_px / scale)) | 1
    pad = small_kernel
    canvas = np.zeros((h // scale + 1 + 2 * pad, w // scale + 1 + 2 * pad), dtype=np.uint8)
    shifted = ((contour - np.array([x, y])) // scale + pad).astype(np.int32)
    cv2.drawContours(canvas, [shifted], -1, 255, thickness=-1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (small_kernel, small_kernel))
    closed = cv2.morphologyEx(canvas, cv2.MORPH_CLOSE, kernel)
    outlines, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    small = max(outlines, key=cv2.contourArea)
    outline = ((small - pad) * scale + np.array([x, y])).astype(np.int32)
    filled = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(filled, [outline], -1, 1, thickness=-1, offset=(-x, -y))
    return outline, filled > 0


def _shape(cv2: Any, outline: Any, touches_border: bool) -> tuple[float, float | None]:
    """(solidity, ellipse fill) of an outline; fill is None at the border or with < 5 points."""
    area = cv2.contourArea(outline)
    hull_area = cv2.contourArea(cv2.convexHull(outline))
    solidity = area / hull_area if hull_area > 0 else 0.0
    if touches_border or len(outline) < ELLIPSE_MIN_POINTS:
        return solidity, None
    _, (axis_a, axis_b), _ = cv2.fitEllipse(outline)
    ellipse_area = math.pi * axis_a * axis_b / 4
    return solidity, (area / ellipse_area if ellipse_area > 0 else 0.0)


def _rejection(candidate: Candidate, params: DetectorParams) -> str | None:
    low_aspect, high_aspect = params.aspect_range
    low_fill, high_fill = params.ellipse_fill_range
    if candidate.area_px < params.min_area_px:
        return "area"
    if not low_aspect <= candidate.aspect <= high_aspect:
        return "aspect"
    if candidate.solidity < params.min_solidity:
        return "solidity"
    if not candidate.touches_border and (candidate.fill is None or not low_fill <= candidate.fill <= high_fill):
        return "ellipse fill"
    if sum(count for count, _ in candidate.spots.values()) < params.min_spots:
        return "spots"
    return None


def _candidate(cv2: Any, contour: Any, masks: _Masks, params: DetectorParams) -> Candidate:
    """Measure one outer contour; cheap rules first, so small clutter skips the repair."""
    x, y, w, h = cv2.boundingRect(contour)
    height, width = masks.components.shape
    margin = params.border_margin_px
    touches_border = x < margin or y < margin or x + w > width - margin or y + h > height - margin
    base = Candidate(x, y, w, h, int(cv2.contourArea(contour)), w / h, 0.0, None, touches_border, {}, None)
    early = _rejection(replace(base, solidity=1.0, fill=1.0, spots={"": (params.min_spots, 0)}), params)
    if early is not None:
        return replace(base, rejected=early)
    outline, inside = _repaired(cv2, contour, (x, y, w, h), params)
    solidity, fill = _shape(cv2, outline, touches_border)
    crops = {color: mask[y:y + h, x:x + w] for color, mask in masks.spots.items()}
    measured = replace(base, solidity=solidity, fill=fill, spots=_spot_pixels(cv2, inside, crops, params))
    return replace(measured, rejected=_rejection(measured, params))


def _as_egg(candidate: Candidate, width: int, height: int) -> EggDetection:
    total = sum(count for count, _ in candidate.spots.values())
    color = max(candidate.spots, key=lambda name: candidate.spots[name][1])
    return EggDetection(cx=(candidate.x + candidate.w / 2) / width, cy=(candidate.y + candidate.h / 2) / height,
                        w=candidate.w / width, h=candidate.h / height, color=color, spots=total,
                        area_px=candidate.area_px, touches_border=candidate.touches_border,
                        solidity=candidate.solidity)


def _frame(frame_bgr: Any) -> Any:
    frame = np.asarray(frame_bgr, dtype=np.uint8)
    if frame.ndim != 3 or frame.shape[2] != 3 or 0 in frame.shape:
        raise ValueError(f"Expected an HxWx3 BGR frame, got shape {frame.shape}")
    return frame


def inspect_candidates(frame_bgr: Any, params: DetectorParams = DEFAULT_PARAMS) -> tuple[Candidate, ...]:
    """Every outer contour after the open/close stage, measured, largest first (for --debug)."""
    cv2 = _cv2()
    masks = _masks(cv2, _frame(frame_bgr), params)
    contours, _ = cv2.findContours(masks.components, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    candidates = (_candidate(cv2, contour, masks, params) for contour in contours)
    return tuple(sorted(candidates, key=lambda candidate: candidate.area_px, reverse=True))


def detect_eggs(frame_bgr: Any, params: DetectorParams = DEFAULT_PARAMS) -> tuple[EggDetection, ...]:
    """All eggs in an HxWx3 uint8 BGR frame, largest (by contour area) first."""
    height, width = _frame(frame_bgr).shape[:2]
    return tuple(_as_egg(candidate, width, height) for candidate in inspect_candidates(frame_bgr, params)
                 if candidate.rejected is None)


def best_egg(frame_bgr: Any, params: DetectorParams = DEFAULT_PARAMS) -> EggDetection | None:
    """The largest detection, or None."""
    detections = detect_eggs(frame_bgr, params)
    return detections[0] if detections else None
