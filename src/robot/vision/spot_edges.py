"""src/robot/vision/spot_edges.py: Edge-based spot stage for the egg detector (EdgeDrawing ellipses).

Alternative to the HSV spot blobs in egg_masks.py, selected with EGG_SPOT_DETECTOR = "edge". It
runs OpenCV contrib's EdgeDrawing (cv2.ximgproc, from opencv-contrib-python-headless) on the gray
frame, keeps the detected circles and ellipses whose inside is at least EGG_EDGE_SPOT_MIN_FILL of
one spot color (HSV mask), and returns them as SpotBlob records, so clustering and segmentation
are shared. Being edge-based it is less sensitive to lighting, but on the check captures it finds
fewer spots than the HSV stage (see config_vision.py). Returns None when cv2.ximgproc is missing,
and the detector then falls back to the HSV stage.
"""

import math
from typing import Any

import numpy as np

from robot.vision.egg_masks import DetectorParams, FrameMasks, SpotBlob

MIN_SEMI_AXIS_PX = 2.0


def edge_available(cv2: Any) -> bool:
    """True when this OpenCV build has the contrib EdgeDrawing module."""
    return hasattr(cv2, "ximgproc") and hasattr(cv2.ximgproc, "createEdgeDrawing")


def _detect_ellipses(cv2: Any, frame: Any) -> Any:
    drawing = cv2.ximgproc.createEdgeDrawing()
    drawing.setParams(cv2.ximgproc.EdgeDrawing.Params())
    drawing.detectEdges(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    ellipses = drawing.detectEllipses()
    return () if ellipses is None else ellipses[:, 0]


def _verified(cv2: Any, ellipse: Any, masks: FrameMasks, params: DetectorParams) -> SpotBlob | None:
    """The ellipse as a SpotBlob when one spot color fills enough of it, else None."""
    cx, cy, radius, semi_a, semi_b, angle = (float(value) for value in ellipse)
    if radius > 0:
        semi_a = semi_b = radius
    if min(semi_a, semi_b) < MIN_SEMI_AXIS_PX:
        return None
    height, width = masks.shape
    reach = int(math.ceil(max(semi_a, semi_b))) + 1
    left, top = max(0, int(cx) - reach), max(0, int(cy) - reach)
    right, bottom = min(width, int(cx) + reach + 1), min(height, int(cy) + reach + 1)
    if right <= left or bottom <= top:
        return None
    inside = np.zeros((bottom - top, right - left), dtype=np.uint8)
    cv2.ellipse(inside, ((cx - left, cy - top), (2 * semi_a, 2 * semi_b), angle), 255, -1)
    area = int(np.count_nonzero(inside))
    if area < params.min_spot_area_px:
        return None
    fills = {color: np.count_nonzero((mask[top:bottom, left:right] > 0) & (inside > 0)) / area
             for color, mask in masks.spots.items()}
    color = max(fills, key=lambda name: fills[name])
    if fills[color] < params.edge_spot_min_fill:
        return None
    x, y, w, h = cv2.boundingRect(inside)
    return SpotBlob(color, cx, cy, area, 2 * math.sqrt(area / math.pi), (left + x, top + y, w, h))


def edge_spot_blobs(cv2: Any, frame: Any, masks: FrameMasks, params: DetectorParams) -> tuple[SpotBlob, ...] | None:
    """Color-verified EdgeDrawing ellipses as spots, or None without cv2.ximgproc."""
    if not edge_available(cv2):
        return None
    candidates = (_verified(cv2, ellipse, masks, params) for ellipse in _detect_ellipses(cv2, frame))
    return tuple(blob for blob in candidates if blob is not None)
