"""src/robot/vision/egg_detector.py: Spot-anchored, scale-adaptive egg detector for the front camera.

Eggs are white ellipsoids with green, red, or orange spots (owner decisions, 2026-09-24). White
alone is not enough (tarp glints under strong light are as white, capture 20260924-223853), so the
spots anchor the search: spot blobs (spot_edges.py or egg_masks.py) are clustered, and each
cluster's median spot diameter d sets the scale. In a window around the cluster, the body OR the
cluster's spot color is closed (the shell's crack) and opened with ~d / 2 (glints thinner than
half a spot); the component holding the spots, completed by missed spots, is gap-repaired and must
pass area, aspect, scale, solidity, ellipse fill (off the border), and spot count and fraction.
The color with the most spot pixels names the egg; best_egg is the largest (nearest) egg.
OpenCV is imported lazily.
"""

from dataclasses import dataclass, replace
import math
from typing import Any

import numpy as np

from robot.vision.egg_masks import (
    DEFAULT_PARAMS,
    DetectorParams,
    FrameMasks,
    SpotBlob,
    SpotCluster,
    cluster_spots,
    frame_masks,
    spot_blobs,
)
from robot.vision import egg_refine as refine
from robot.vision.egg_size import EggDetection
from robot.vision.spot_edges import edge_spot_blobs

__all__ = ("Candidate", "DetectorParams", "EggDetection", "best_egg", "detect_eggs", "inspect_candidates")

ELLIPSE_MIN_POINTS = 5  # cv2.fitEllipse needs at least five contour points
KERNEL_MAX_PX = 15  # morphology runs on a downscaled mask so kernels stay about this size (speed)
DUPLICATE_IOU = 0.5  # two clusters that found the same egg: keep the larger


@dataclass(frozen=True)
class Candidate:
    """One spot cluster's egg hypothesis with every measurement and the rule that rejected it
    (`rejected` is None for an egg). Pixel bbox; solidity and fill are measured after gap repair."""

    cluster_spots: int
    d_med: float
    window: tuple[int, int, int, int]
    x: int = 0
    y: int = 0
    w: int = 0
    h: int = 0
    area_px: int = 0
    aspect: float = 0.0
    scale: float = 0.0
    solidity: float = 0.0
    fill: float | None = None
    touches_border: bool = False
    spots: tuple[tuple[str, int, int], ...] = ()  # (color, blobs, pixels) inside the egg
    ellipse: tuple[float, float, float, float, float] | None = None  # pixels: cx, cy, axis1, axis2, angle
    stages: tuple = ()  # (stage, frame bbox) of the component, filled when tracing (--debug)
    rejected: str | None = None


def spot_fraction(candidate: Candidate) -> float:
    """Spot pixels inside the egg / egg area (0 before measurement)."""
    return sum(pixels for _, _, pixels in candidate.spots) / candidate.area_px if candidate.area_px else 0.0


def _cv2() -> Any:
    import cv2  # lazy: see the module docstring

    return cv2


def _odd(value: float) -> int:
    return max(3, int(round(value))) | 1


def _morph(cv2: Any, mask: Any, operation: int, size: int) -> Any:
    return cv2.morphologyEx(mask, operation, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size)))


def _window(cluster: SpotCluster, shape: tuple[int, int], params: DetectorParams) -> tuple[int, int, int, int]:
    grow = int(params.window_factor * cluster.d_med)
    x, y, w, h = cluster.box
    left, top = max(0, x - grow), max(0, y - grow)
    return left, top, min(shape[1], x + w + grow) - left, min(shape[0], y + h + grow) - top


def _segment(cv2: Any, masks: FrameMasks, cluster: SpotCluster, window: tuple[int, int, int, int],
             params: DetectorParams, trace: bool = False) -> tuple[Any | None, tuple]:
    """Window mask of the component holding the cluster's spots (or None), and per-stage bboxes when
    `trace` is set (--debug). Close (crack), edge fence outside the spot hull, open (~d/2, glints),
    spot vote, then missed spots and the hull cap (egg_refine.py)."""
    wx, wy, ww, wh = window
    colors = {spot.color for spot in cluster.core}  # one color per cluster: other colors stay out
    spots = np.zeros((wh, ww), dtype=np.uint8)
    for color in sorted(colors):
        spots = cv2.bitwise_or(spots, masks.spots[color][wy:wy + wh, wx:wx + ww])
    square = cv2.getStructuringElement(cv2.MORPH_RECT, (_odd(params.close_kernel_px),) * 2)  # separable: fast
    full = cv2.morphologyEx(cv2.bitwise_or(masks.body[wy:wy + wh, wx:wx + ww], spots), cv2.MORPH_CLOSE, square)
    spot_pixels = refine.core_spot_pixels(spots, cluster, (wx, wy))
    fenced = full.copy()
    fenced[refine.fence(cv2, masks, window, params) & ~refine.spot_hull(cv2, spot_pixels)] = 0
    chosen = _pick(cv2, fenced, cluster, window, params)
    if chosen is None:
        return None, ()
    component = refine.regrow(cv2, (chosen & (fenced > 0)) | spot_pixels, full, params)
    missed = refine.missed_spots(cv2, masks, component, cluster, window, params)
    capped = (component | missed) & refine.hull_cap(cv2, cluster, window, missed, params)
    stages = ()
    if trace:
        loose = _pick(cv2, full, cluster, window, params)
        stages = (("no fence", refine.box_of(loose & (full > 0), (wx, wy)) if loose is not None else (0, 0, 0, 0)),
                  ("fenced", refine.box_of(component, (wx, wy))), ("capped", refine.box_of(capped, (wx, wy))))
    return capped, stages


def _pick(cv2: Any, mask: Any, cluster: SpotCluster, window: tuple[int, int, int, int],
          params: DetectorParams) -> Any | None:
    """Open (~d/2) at reduced scale; the component touched by the most core spot rings, full res."""
    wx, wy, ww, wh = window
    open_px = params.open_spot_factor * cluster.d_med
    scale = max(1, math.ceil(open_px / KERNEL_MAX_PX))
    small = cv2.resize(mask, (max(1, ww // scale), max(1, wh // scale)), interpolation=cv2.INTER_NEAREST)
    small = _morph(cv2, small, cv2.MORPH_OPEN, _odd(open_px / scale))
    _, labels, stats, _ = cv2.connectedComponentsWithStats(small, connectivity=8)
    reach = 2 * params.fence_dilate_px + open_px / 2  # past the fence band and the open's erosion
    areas = stats[:, cv2.CC_STAT_AREA]
    best = refine.ring_vote(labels, areas, cluster, (wx, wy), scale, reach)
    if best == 0:
        return None
    return cv2.resize((labels == best).astype(np.uint8), (ww, wh), interpolation=cv2.INTER_NEAREST) > 0


def _repaired(cv2: Any, component: Any, params: DetectorParams) -> tuple[Any, Any]:
    """(outer contour, filled mask) of `component` after a gap-repair close proportional to its bbox."""
    points = cv2.findNonZero(component.astype(np.uint8))
    x, y, w, h = cv2.boundingRect(points)
    kernel_px = max(params.close_kernel_px, params.repair_kernel_fraction * min(w, h))
    scale = max(1, math.ceil(kernel_px / KERNEL_MAX_PX))
    pad = _odd(kernel_px / scale)
    crop = component[y:y + h, x:x + w].astype(np.uint8) * 255
    small = cv2.resize(crop, (max(1, w // scale), max(1, h // scale)), interpolation=cv2.INTER_NEAREST)
    small = cv2.copyMakeBorder(small, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=0)
    closed = _morph(cv2, small, cv2.MORPH_CLOSE, pad)
    outlines, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    outline = ((max(outlines, key=cv2.contourArea) - pad) * scale + np.array([x, y])).astype(np.int32)
    filled = np.zeros(component.shape, dtype=np.uint8)
    cv2.drawContours(filled, [outline], -1, 1, thickness=-1)
    return outline, (filled > 0) | component


def _shape(cv2: Any, outline: Any, touches_border: bool) -> tuple[float, float | None, tuple | None]:
    """(solidity, ellipse fill, fitted ellipse in pixels) of an outline. The ellipse needs >= 5
    points; the fill is None at the frame border (a truncated egg is not a full ellipse)."""
    area = cv2.contourArea(outline)
    hull_area = cv2.contourArea(cv2.convexHull(outline))
    solidity = area / hull_area if hull_area > 0 else 0.0
    if len(outline) < ELLIPSE_MIN_POINTS:
        return solidity, None, None
    (center_x, center_y), (axis_a, axis_b), angle = cv2.fitEllipse(outline)
    ellipse = (float(center_x), float(center_y), float(axis_a), float(axis_b), float(angle))
    ellipse_area = math.pi * axis_a * axis_b / 4
    fill = None if touches_border else (area / ellipse_area if ellipse_area > 0 else 0.0)
    return solidity, fill, ellipse


def _rejection(candidate: Candidate, params: DetectorParams, shaped: bool) -> str | None:
    """The first rule the candidate breaks; shape and spot rules only once they are measured."""
    low_aspect, high_aspect = params.aspect_range
    low_scale, high_scale = params.scale_range
    low_fill, high_fill = params.ellipse_fill_range
    cheap = ((candidate.area_px < params.min_area_px, "area"),
             (not low_aspect <= candidate.aspect <= high_aspect, "aspect"),
             (not low_scale <= candidate.scale <= high_scale, "scale"))
    shape = ((candidate.solidity < params.min_solidity, "solidity"),
             (not candidate.touches_border and (candidate.fill is None or not low_fill <= candidate.fill <= high_fill),
              "ellipse fill"),
             (sum(blobs for _, blobs, _ in candidate.spots) < params.min_spots, "spots"),
             (spot_fraction(candidate) < params.min_spot_fraction, "spot fraction"))
    return next((reason for broken, reason in cheap + (shape if shaped else ()) if broken), None)


def _spot_counts(cv2: Any, masks: FrameMasks, inside: Any, window: tuple[int, int, int, int],
                 params: DetectorParams, cluster_colors: set[str]) -> tuple[tuple[str, int, int], ...]:
    """(color, spot blobs, spot pixels) inside the egg from the HSV color masks, so spots the spot
    stage missed count too; only the cluster's own color is counted (others are 0)."""
    wx, wy, ww, wh = window
    counts = []
    for color, mask in masks.colors.items():
        if color not in cluster_colors:
            counts.append((color, 0, 0))
            continue
        blob_mask = ((mask[wy:wy + wh, wx:wx + ww] > 0) & inside).astype(np.uint8)
        total, _, stats, _ = cv2.connectedComponentsWithStats(blob_mask, connectivity=8)
        kept = [int(stats[index, cv2.CC_STAT_AREA]) for index in range(1, total)
                if stats[index, cv2.CC_STAT_AREA] >= params.min_spot_area_px]
        counts.append((color, len(kept), sum(kept)))
    return tuple(counts)


def _candidate(cv2: Any, masks: FrameMasks, cluster: SpotCluster, params: DetectorParams,
               trace: bool = False) -> Candidate:
    window = _window(cluster, masks.shape, params)
    base = Candidate(cluster_spots=len(cluster.core), d_med=cluster.d_med, window=window)
    if cluster.d_med < params.min_spot_diameter_px:
        return replace(base, rejected="spot size")
    component, stages = _segment(cv2, masks, cluster, window, params, trace)
    if component is None or not component.any():
        return replace(base, rejected="no component")
    wx, wy = window[:2]
    x, y, w, h = cv2.boundingRect(cv2.findNonZero(component.astype(np.uint8)))
    height, width = masks.shape
    margin = params.border_margin_px
    touches = wx + x < margin or wy + y < margin or wx + x + w > width - margin or wy + y + h > height - margin
    measured = replace(base, x=wx + x, y=wy + y, w=w, h=h, area_px=int(component.sum()), aspect=w / h,
                       scale=h / cluster.d_med, touches_border=touches, stages=stages)
    early = _rejection(measured, params, shaped=False)
    if early is not None:
        return replace(measured, rejected=early)
    outline, inside = _repaired(cv2, component, params)
    solidity, fill, ellipse = _shape(cv2, outline + np.array([wx, wy], dtype=np.int32), touches)
    shaped = replace(measured, solidity=solidity, fill=fill, ellipse=ellipse,
                     spots=_spot_counts(cv2, masks, inside, window, params, {spot.color for spot in cluster.core}))
    return replace(shaped, rejected=_rejection(shaped, params, shaped=True))


def _iou(a: Candidate, b: Candidate) -> float:
    left, top = max(a.x, b.x), max(a.y, b.y)
    right, bottom = min(a.x + a.w, b.x + b.w), min(a.y + a.h, b.y + b.h)
    overlap = max(0, right - left) * max(0, bottom - top)
    union = a.w * a.h + b.w * b.h - overlap
    return overlap / union if union > 0 else 0.0


def _frame(frame_bgr: Any) -> Any:
    frame = np.asarray(frame_bgr, dtype=np.uint8)
    if frame.ndim != 3 or frame.shape[2] != 3 or 0 in frame.shape:
        raise ValueError(f"Expected an HxWx3 BGR frame, got shape {frame.shape}")
    return frame


def _spots(cv2: Any, frame: Any, masks: FrameMasks, params: DetectorParams) -> tuple[FrameMasks, tuple[SpotBlob, ...]]:
    """The configured spot stage. "edge" also replaces the spot masks with the accepted spot
    interiors (so tarp of a spot's color never joins an egg); it falls back to "hsv" without
    cv2.ximgproc."""
    if params.spot_detector == "edge":
        found = edge_spot_blobs(cv2, frame, masks, params)
        if found is not None:
            return replace(masks, spots=found.interiors), found.blobs
    elif params.spot_detector != "hsv":
        raise ValueError(f"Unknown spot detector: {params.spot_detector!r}")
    return masks, spot_blobs(cv2, masks, params)


def inspect_candidates(
    frame_bgr: Any, params: DetectorParams = DEFAULT_PARAMS, trace: bool = False
) -> tuple[Candidate, ...]:
    """One measured candidate per spot cluster, largest cluster first; `trace` also records the
    component bbox per stage (no fence / fenced / capped) for --debug."""
    cv2 = _cv2()
    frame = _frame(frame_bgr)
    masks = frame_masks(cv2, frame, params)
    masks, blobs = _spots(cv2, frame, masks, params)
    clusters = cluster_spots(blobs, params.spot_cluster_factor, params.core_spot_factor)
    return tuple(_candidate(cv2, masks, cluster, params, trace) for cluster in clusters)


def _as_egg(candidate: Candidate, width: int, height: int) -> EggDetection:
    color, _, _ = max(candidate.spots, key=lambda entry: entry[2])
    ellipse = None
    if candidate.ellipse is not None:
        center_x, center_y, axis_a, axis_b, angle = candidate.ellipse
        ellipse = (center_x / width, center_y / height, axis_a / width, axis_b / height, angle)
    return EggDetection(cx=(candidate.x + candidate.w / 2) / width, cy=(candidate.y + candidate.h / 2) / height,
                        w=candidate.w / width, h=candidate.h / height, color=color,
                        spots=sum(blobs for _, blobs, _ in candidate.spots), area_px=candidate.area_px,
                        touches_border=candidate.touches_border, solidity=candidate.solidity, ellipse=ellipse)


def detect_eggs(frame_bgr: Any, params: DetectorParams = DEFAULT_PARAMS) -> tuple[EggDetection, ...]:
    """All eggs in an HxWx3 uint8 BGR frame, largest (by area) first; duplicates merged."""
    height, width = _frame(frame_bgr).shape[:2]
    eggs = sorted((candidate for candidate in inspect_candidates(frame_bgr, params) if candidate.rejected is None),
                  key=lambda candidate: candidate.area_px, reverse=True)
    kept: tuple[Candidate, ...] = ()
    for candidate in eggs:
        if all(_iou(candidate, other) < DUPLICATE_IOU for other in kept):
            kept = kept + (candidate,)
    return tuple(_as_egg(candidate, width, height) for candidate in kept)


def best_egg(frame_bgr: Any, params: DetectorParams = DEFAULT_PARAMS) -> EggDetection | None:
    """The largest detection, or None."""
    detections = detect_eggs(frame_bgr, params)
    return detections[0] if detections else None
