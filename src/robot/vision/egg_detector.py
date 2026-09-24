"""src/robot/vision/egg_detector.py: Color-based egg detector for the front camera (Auto Catch).

Eggs are white ellipsoids with green, blue, or red spots (owner decision, 2026-09-24). Pipeline
per BGR frame: HSV -> white-body mask OR the spot masks -> morphological close -> connected
components -> keep components with enough area and an egg-like bbox aspect -> count the spot
blobs of each color inside the component; at least EGG_MIN_SPOTS are required and the color
with the most spot pixels names the egg. Thresholds are placeholders in config.py (calibrate at
the venue). OpenCV is imported lazily so importing this module never loads cv2 (see __init__).
"""

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from robot.config import (
    EGG_ASPECT_RANGE,
    EGG_BODY_HSV,
    EGG_CLOSE_KERNEL_PX,
    EGG_MIN_AREA_PX,
    EGG_MIN_SPOT_AREA_PX,
    EGG_MIN_SPOTS,
    EGG_SPOT_HSV,
)
from robot.vision.egg_size import EggDetection

__all__ = ("DetectorParams", "EggDetection", "best_egg", "detect_eggs")

HsvRange = tuple[tuple[int, int, int], tuple[int, int, int]]


@dataclass(frozen=True)
class DetectorParams:
    """All detector thresholds; the defaults come from config.py."""

    body_hsv: HsvRange = EGG_BODY_HSV
    spot_hsv: Mapping[str, tuple[HsvRange, ...]] = field(default_factory=lambda: EGG_SPOT_HSV)
    min_area_px: int = EGG_MIN_AREA_PX
    aspect_range: tuple[float, float] = EGG_ASPECT_RANGE
    min_spots: int = EGG_MIN_SPOTS
    close_kernel_px: int = EGG_CLOSE_KERNEL_PX
    min_spot_area_px: int = EGG_MIN_SPOT_AREA_PX


DEFAULT_PARAMS = DetectorParams()


def _cv2() -> Any:
    import cv2  # lazy: see the module docstring

    return cv2


def _in_ranges(cv2: Any, hsv: Any, ranges: tuple[HsvRange, ...]) -> Any:
    masks = [cv2.inRange(hsv, np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8)) for low, high in ranges]
    return np.bitwise_or.reduce(masks) if masks else np.zeros(hsv.shape[:2], dtype=np.uint8)


def _spot_blobs(cv2: Any, mask: Any, min_area: int) -> tuple[int, int]:
    """(number of blobs with area >= min_area, their total pixels) in a 0/1 mask."""
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    areas = [int(stats[index, cv2.CC_STAT_AREA]) for index in range(1, count)]
    kept = [area for area in areas if area >= min_area]
    return len(kept), sum(kept)


def _classify_component(
    cv2: Any, component: Any, spot_masks: Mapping[str, Any], params: DetectorParams
) -> tuple[str, int] | None:
    """(color, total spot blobs) for one component crop, or None when it has too few spots."""
    blobs = {color: _spot_blobs(cv2, ((mask > 0) & component).astype(np.uint8), params.min_spot_area_px)
             for color, mask in spot_masks.items()}
    total = sum(count for count, _ in blobs.values())
    if total < params.min_spots:
        return None
    color = max(blobs, key=lambda name: blobs[name][1])
    return color, total


def _component_egg(
    cv2: Any, index: int, stats: Any, labels: Any, spot_masks: Mapping[str, Any], params: DetectorParams
) -> EggDetection | None:
    """The egg for one connected component, or None when its area, aspect, or spots do not fit."""
    x, y, w, h, area = (int(value) for value in stats[index])
    low_aspect, high_aspect = params.aspect_range
    if area < params.min_area_px or not low_aspect <= w / h <= high_aspect:
        return None
    crops = {color: mask[y:y + h, x:x + w] for color, mask in spot_masks.items()}
    spots = _classify_component(cv2, labels[y:y + h, x:x + w] == index, crops, params)
    if spots is None:
        return None
    height, width = labels.shape
    return EggDetection(cx=(x + w / 2) / width, cy=(y + h / 2) / height, w=w / width, h=h / height,
                        color=spots[0], spots=spots[1], area_px=area)


def detect_eggs(frame_bgr: Any, params: DetectorParams = DEFAULT_PARAMS) -> tuple[EggDetection, ...]:
    """All eggs in an HxWx3 uint8 BGR frame, largest (by component area) first."""
    cv2 = _cv2()
    frame = np.asarray(frame_bgr, dtype=np.uint8)
    if frame.ndim != 3 or frame.shape[2] != 3 or 0 in frame.shape:
        raise ValueError(f"Expected an HxWx3 BGR frame, got shape {frame.shape}")
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    spot_masks = {color: _in_ranges(cv2, hsv, ranges) for color, ranges in params.spot_hsv.items()}
    union = np.bitwise_or.reduce([_in_ranges(cv2, hsv, (params.body_hsv,)), *spot_masks.values()])
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (params.close_kernel_px,) * 2)
    closed = cv2.morphologyEx(union, cv2.MORPH_CLOSE, kernel)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(closed, connectivity=8)
    eggs = (_component_egg(cv2, index, stats, labels, spot_masks, params) for index in range(1, count))
    return tuple(sorted((egg for egg in eggs if egg is not None), key=lambda det: det.area_px, reverse=True))


def best_egg(frame_bgr: Any, params: DetectorParams = DEFAULT_PARAMS) -> EggDetection | None:
    """The largest detection, or None."""
    detections = detect_eggs(frame_bgr, params)
    return detections[0] if detections else None
