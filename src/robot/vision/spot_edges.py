"""src/robot/vision/spot_edges.py: Edge-based spot stage for the egg detector (the default).

Color alone cannot find spots: the blue egg's spots had the blue tarp's color (robot run
2026-09-24, captures 20260924-231156/-231202; blue eggs were then dropped), and strong light makes
tarp glints as white as the egg. The white-ring test below stays for that robustness. This stage runs OpenCV contrib's
EdgeDrawing (cv2.ximgproc, from opencv-contrib-python-headless) on the gray frame and keeps each
circle or ellipse that (a) is at least EGG_MIN_SPOT_DIAMETER_PX across its minor axis, (b) is
surrounded by white: in the ring from EGG_RING_SCALES[0] to [1] times its axes, at least
EGG_RING_WHITE_FRACTION of the in-frame pixels are egg white (EGG_RING_WHITE_HSV), which is what
tells a spot on an egg from a patch of tarp, and (c) is mostly one spot color inside
(EGG_EDGE_SPOT_MIN_FILL of its pixels in one EGG_SPOT_HSV range, basket rules applied). The
accepted interiors (their pixels of that color), drawn per color, replace the HSV spot masks when
the egg component is built, so tarp of the spot's color never joins an egg. Returns None without
cv2.ximgproc; the detector
then falls back to the HSV blob stage.
"""

from collections.abc import Mapping
from dataclasses import dataclass
import math
from typing import Any

import numpy as np

from robot.vision.egg_masks import DetectorParams, FrameMasks, SpotBlob, in_ranges


@dataclass(frozen=True)
class EdgeSpots:
    """Accepted spots and their filled interiors per color (0/255 frame masks)."""

    blobs: tuple[SpotBlob, ...]
    interiors: Mapping[str, Any]


@dataclass(frozen=True)
class _Ellipse:
    cx: float
    cy: float
    semi_a: float
    semi_b: float
    angle: float


def edge_available(cv2: Any) -> bool:
    """True when this OpenCV build has the contrib EdgeDrawing module."""
    return hasattr(cv2, "ximgproc") and hasattr(cv2.ximgproc, "createEdgeDrawing")


def _detect_ellipses(cv2: Any, frame: Any) -> tuple[_Ellipse, ...]:
    drawing = cv2.ximgproc.createEdgeDrawing()
    drawing.setParams(cv2.ximgproc.EdgeDrawing.Params())
    drawing.detectEdges(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    found = drawing.detectEllipses()
    if found is None:
        return ()
    # Rows are (cx, cy, radius, semi_a, semi_b, angle): circles set radius, ellipses the semi-axes.
    return tuple(_Ellipse(float(cx), float(cy), float(r or a), float(r or b), float(angle))
                 for cx, cy, r, a, b, angle in found[:, 0])


def _draw(cv2: Any, shape: tuple[int, int], origin: tuple[int, int], ellipse: _Ellipse, scale: float) -> Any:
    mask = np.zeros(shape, dtype=np.uint8)
    center = (ellipse.cx - origin[0], ellipse.cy - origin[1])
    cv2.ellipse(mask, (center, (2 * ellipse.semi_a * scale, 2 * ellipse.semi_b * scale), ellipse.angle), 255, -1)
    return mask > 0


def _check(cv2: Any, ellipse: _Ellipse, masks: FrameMasks, white: Any,
           params: DetectorParams) -> tuple[str, tuple[int, int, int, int]] | None:
    """(color, interior box in the ROI) when the ellipse passes size, ring, and color checks."""
    if 2 * min(ellipse.semi_a, ellipse.semi_b) < params.min_spot_diameter_px:
        return None
    height, width = masks.shape
    outer = params.ring_scales[1]
    reach = int(math.ceil(max(ellipse.semi_a, ellipse.semi_b) * outer)) + 1
    left, top = max(0, int(ellipse.cx) - reach), max(0, int(ellipse.cy) - reach)
    right, bottom = min(width, int(ellipse.cx) + reach + 1), min(height, int(ellipse.cy) + reach + 1)
    if right <= left or bottom <= top:
        return None
    shape, origin = (bottom - top, right - left), (left, top)
    inside = _draw(cv2, shape, origin, ellipse, 1.0)
    area = int(np.count_nonzero(inside))
    ring = _draw(cv2, shape, origin, ellipse, outer) & ~_draw(cv2, shape, origin, ellipse, params.ring_scales[0])
    ring_px = int(np.count_nonzero(ring))
    if area < params.min_spot_area_px or ring_px == 0:
        return None
    if np.count_nonzero(ring & white[top:bottom, left:right]) / ring_px < params.ring_white_fraction:
        return None
    fills = {color: np.count_nonzero(inside & (mask[top:bottom, left:right] > 0)) / area
             for color, mask in masks.spots.items()}
    color = max(fills, key=lambda name: fills[name])
    return (color, origin) if fills[color] >= params.edge_spot_min_fill else None


def _paint(cv2: Any, ellipse: _Ellipse, color: str, masks: FrameMasks, interiors: Mapping[str, Any]) -> SpotBlob | None:
    """Add the ellipse's pixels of `color` (an overshooting ellipse adds no shadow or tarp) to the
    interior mask, in place on this function's own buffers; return the spot, or None if empty."""
    height, width = masks.shape
    reach = int(math.ceil(max(ellipse.semi_a, ellipse.semi_b))) + 1
    left, top = max(0, int(ellipse.cx) - reach), max(0, int(ellipse.cy) - reach)
    right, bottom = min(width, int(ellipse.cx) + reach + 1), min(height, int(ellipse.cy) + reach + 1)
    colored = masks.spots[color][top:bottom, left:right] > 0
    inside = _draw(cv2, (bottom - top, right - left), (left, top), ellipse, 1.0) & colored
    area = int(np.count_nonzero(inside))
    if area == 0:
        return None
    interiors[color][top:bottom, left:right] |= inside.astype(np.uint8) * 255
    x, y, w, h = cv2.boundingRect(inside.astype(np.uint8))
    return SpotBlob(color, ellipse.cx, ellipse.cy, area, 2 * math.sqrt(area / math.pi), (left + x, top + y, w, h))


def edge_spot_blobs(cv2: Any, frame: Any, masks: FrameMasks, params: DetectorParams) -> EdgeSpots | None:
    """White-ringed, color-classified EdgeDrawing ellipses as spots, or None without cv2.ximgproc."""
    if not edge_available(cv2):
        return None
    hsv = masks.hsv if masks.hsv is not None else cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    white = in_ranges(cv2, hsv, params.ring_white_hsv) > 0
    height, width = masks.shape
    interiors = {color: np.zeros((height, width), dtype=np.uint8) for color in masks.spots}
    blobs = []
    for ellipse in _detect_ellipses(cv2, frame):
        verdict = _check(cv2, ellipse, masks, white, params)
        if verdict is None:
            continue
        blob = _paint(cv2, ellipse, verdict[0], masks, interiors)
        if blob is not None:
            blobs.append(blob)
    return EdgeSpots(blobs=tuple(blobs), interiors=interiors)
