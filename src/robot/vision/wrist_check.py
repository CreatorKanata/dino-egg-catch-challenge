"""src/robot/vision/wrist_check.py: Is the egg in the wrist camera view at the catch pose? (Auto Catch, Phase 3)

At the catch pose the head points down and the wrist camera (640x480) sees the aligned egg close
up, cut by the frame border, next to the white gripper parts, a teal gripper part, the pink basket
edge, and a bright tarp (captures 20260925-014442 near egg, -014418 far egg out of view, -014506
no egg). The front camera's egg-shape rules (scale, solidity, ellipse fill) do not fit such a
view, so this is a spot-only close-up mode: the egg is present when at least WRIST_MIN_SPOTS spots
are found, each an EdgeDrawing ellipse or an HSV color blob (ellipse-fitted) that is disk-like
(minor axis >= WRIST_MIN_SPOT_PX, axis ratio <= WRIST_MAX_SPOT_ASPECT), filled with one egg color
(>= WRIST_SPOT_MIN_FILL), and surrounded by egg white on >= WRIST_RING_WHITE_FRACTION of the
in-frame part of its ring (>= WRIST_RING_MIN_INSIDE of the ring inside the frame). The white
gripper parts have no spots; the teal part and the basket edge are not disks and have no white
ring. The WristView gives the color of the largest accepted spot, the centroid of the accepted
spots, and `partial` when an accepted spot or the white region around it touches the border. The
loop calls this only during Auto Catch's wrist_check frames, when WRIST_CHECK_ENABLED.
OpenCV is imported lazily.
"""

from dataclasses import dataclass
import math
from typing import Any

import numpy as np

from robot.vision.config_vision import (
    WRIST_MAX_SPOT_ASPECT,
    WRIST_MIN_SPOT_PX,
    WRIST_MIN_SPOTS,
    WRIST_RING_MIN_INSIDE,
    WRIST_RING_WHITE_FRACTION,
    WRIST_SPOT_MIN_FILL,
)
from robot.vision.egg_masks import DEFAULT_PARAMS, DetectorParams, FrameMasks, frame_masks, in_ranges
from robot.vision.egg_size import WristView
from robot.vision.spot_edges import edge_available

ELLIPSE_MIN_POINTS = 5


@dataclass(frozen=True)
class WristSpot:
    """One spot candidate with its measures (full axes in px) and whether it counts as a spot."""

    source: str
    cx: float
    cy: float
    axis_a: float
    axis_b: float
    angle: float
    color: str
    color_share: float
    ring_inside: float
    ring_white: float
    touches_border: bool
    accepted: bool


def _frame(frame_bgr: Any) -> Any:
    frame = np.asarray(frame_bgr, dtype=np.uint8)
    if frame.ndim != 3 or frame.shape[2] != 3 or 0 in frame.shape:
        raise ValueError(f"Expected an HxWx3 BGR frame, got shape {frame.shape}")
    return frame


def _edge_ellipses(cv2: Any, frame: Any) -> list[tuple[str, tuple]]:
    drawing = cv2.ximgproc.createEdgeDrawing()
    drawing.setParams(cv2.ximgproc.EdgeDrawing.Params())
    drawing.detectEdges(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    found = drawing.detectEllipses()
    if found is None:
        return []
    return [("edge", ((float(cx), float(cy)), (2 * float(r or a), 2 * float(r or b)), float(angle)))
            for cx, cy, r, a, b, angle in found[:, 0]]


def _blob_ellipses(cv2: Any, masks: FrameMasks) -> list[tuple[str, tuple]]:
    """Ellipses fitted to the outer contours of large enough HSV spot-color blobs."""
    min_area = math.pi / 4 * WRIST_MIN_SPOT_PX ** 2
    return [("hsv", cv2.fitEllipse(outline)) for mask in masks.spots.values()
            for outline in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
            if len(outline) >= ELLIPSE_MIN_POINTS and cv2.contourArea(outline) >= min_area]


def _disk_like(ellipse: tuple) -> bool:
    small, large = sorted(ellipse[1])
    return small >= WRIST_MIN_SPOT_PX and large <= WRIST_MAX_SPOT_ASPECT * small


def _measure(cv2: Any, source: str, ellipse: tuple, masks: FrameMasks, white: Any, params: DetectorParams) -> WristSpot:
    """Interior color share and ring measures, drawn in the region around the ellipse only."""
    (cx, cy), (axis_a, axis_b), angle = ellipse
    height, width = masks.shape
    outer, inner = params.ring_scales[1], params.ring_scales[0]
    reach = int(math.ceil(max(axis_a, axis_b) * outer / 2)) + 1
    left, top = max(0, int(cx) - reach), max(0, int(cy) - reach)
    right, bottom = min(width, int(cx) + reach + 1), min(height, int(cy) + reach + 1)
    local = ((cx - left, cy - top), (axis_a, axis_b), angle)
    interior = np.zeros((bottom - top, right - left), dtype=np.uint8)
    cv2.ellipse(interior, local, 1, -1)
    ring = np.zeros_like(interior)
    cv2.ellipse(ring, (local[0], (axis_a * outer, axis_b * outer), angle), 1, -1)
    cv2.ellipse(ring, (local[0], (axis_a * inner, axis_b * inner), angle), 0, -1)
    area, ring_px = max(1, int(np.count_nonzero(interior))), int(np.count_nonzero(ring))
    shares = {color: np.count_nonzero(mask[top:bottom, left:right][interior > 0]) / area
              for color, mask in masks.spots.items()}
    color = max(shares, key=lambda name: shares[name])
    ring_total = math.pi / 4 * axis_a * axis_b * (outer ** 2 - inner ** 2)
    ring_inside = ring_px / ring_total if ring_total > 0 else 0.0
    ring_white = np.count_nonzero(white[top:bottom, left:right] & (ring > 0)) / ring_px if ring_px else 0.0
    half = max(axis_a, axis_b) / 2
    touches = cx - half <= 0 or cy - half <= 0 or cx + half >= width - 1 or cy + half >= height - 1
    accepted = (shares[color] >= WRIST_SPOT_MIN_FILL and ring_inside >= WRIST_RING_MIN_INSIDE
                and ring_white >= WRIST_RING_WHITE_FRACTION)
    return WristSpot(source, cx, cy, axis_a, axis_b, angle, color, shares[color], ring_inside, ring_white,
                     touches, accepted)


def _white_touches_border(cv2: Any, white: Any, spots: list[WristSpot]) -> bool:
    """Whether the white region right around an accepted spot reaches the frame border (egg cut)."""
    count, labels, stats, _ = cv2.connectedComponentsWithStats(white.astype(np.uint8), connectivity=8)
    height, width = white.shape
    for spot in spots:
        ring = np.zeros((height, width), dtype=np.uint8)
        cv2.ellipse(ring, ((spot.cx, spot.cy), (spot.axis_a * 1.4, spot.axis_b * 1.4), spot.angle), 1, 3)
        for label in np.unique(labels[(ring > 0) & white]):
            x, y, w, h = (int(value) for value in stats[label, :4])
            if label and (x == 0 or y == 0 or x + w >= width or y + h >= height):
                return True
    return False


def wrist_spots(frame_bgr: Any, params: DetectorParams = DEFAULT_PARAMS) -> tuple[WristSpot, ...]:
    """Every disk-like spot candidate (EdgeDrawing ellipses when available, plus HSV blobs)."""
    import cv2  # lazy: see the module docstring

    frame = _frame(frame_bgr)
    masks = frame_masks(cv2, frame, params)
    white = in_ranges(cv2, masks.hsv, params.ring_white_hsv) > 0
    candidates = (_edge_ellipses(cv2, frame) if edge_available(cv2) else []) + _blob_ellipses(cv2, masks)
    return tuple(_measure(cv2, source, ellipse, masks, white, params)
                 for source, ellipse in candidates if _disk_like(ellipse))


def egg_in_wrist_view(frame_bgr: Any, params: DetectorParams = DEFAULT_PARAMS,
                      min_spots: int = WRIST_MIN_SPOTS) -> WristView | None:
    """The egg in an HxWx3 uint8 BGR wrist frame from its accepted spots, or None."""
    import cv2  # lazy: see the module docstring

    frame = _frame(frame_bgr)
    accepted = [spot for spot in wrist_spots(frame, params) if spot.accepted]
    if len(accepted) < min_spots:
        return None
    height, width = frame.shape[:2]
    largest = max(accepted, key=lambda spot: spot.axis_a * spot.axis_b)
    cx = min(1.0, max(0.0, sum(spot.cx for spot in accepted) / len(accepted) / width))
    cy = min(1.0, max(0.0, sum(spot.cy for spot in accepted) / len(accepted) / height))
    white = in_ranges(cv2, cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), params.ring_white_hsv) > 0
    partial = any(spot.touches_border for spot in accepted) or _white_touches_border(cv2, white, accepted)
    return WristView(color=largest.color, partial=partial, cx=cx, cy=cy)
