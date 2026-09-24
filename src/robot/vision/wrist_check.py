"""src/robot/vision/wrist_check.py: Is the egg in the wrist camera view at the catch pose? (Auto Catch, Phase 3)

At the catch pose the head points down and the wrist camera (640x480) sees the aligned egg close
up, often cut by the frame border (docs/spec/operating-modes.md, section 4, step 3). The check runs
the front-camera egg detector (border-touching components are already allowed, the ellipse-fill
test is already skipped at the border, and min_area_px is kept, so WRIST_PARAMS are the detector's
own parameters for now). Without a full detection it accepts a partial egg: a cluster of at least
WRIST_MIN_SPOT_CLUSTER spots of one color, each surrounded by egg white (the default "edge" spot
stage; without cv2.ximgproc the HSV fallback has no white-ring test). The result is a WristView
(color, partial, center of the egg bbox or of the cluster's search window). Only the detector's
public functions are used; the spot stage runs a second time only when no full egg was found. The
loop calls this only during Auto Catch's wrist_check frames, and only when WRIST_CHECK_ENABLED.
OpenCV is imported lazily (see robot/vision/__init__.py).
"""

from typing import Any, Final

import numpy as np

from robot.vision.config_vision import WRIST_MIN_SPOT_CLUSTER
from robot.vision.egg_detector import detect_eggs
from robot.vision.egg_masks import DEFAULT_PARAMS, DetectorParams, SpotBlob, SpotCluster, cluster_spots
from robot.vision.egg_masks import frame_masks, spot_blobs
from robot.vision.egg_size import WristView
from robot.vision.spot_edges import edge_spot_blobs

# The close-up view needs no relaxation beyond the detector's existing border rules (see above);
# tune a copy here (dataclasses.replace) once wrist captures at the catch pose exist.
WRIST_PARAMS: Final = DEFAULT_PARAMS


def _frame(frame_bgr: Any) -> Any:
    frame = np.asarray(frame_bgr, dtype=np.uint8)
    if frame.ndim != 3 or frame.shape[2] != 3 or 0 in frame.shape:
        raise ValueError(f"Expected an HxWx3 BGR frame, got shape {frame.shape}")
    return frame


def _spots(frame: Any, params: DetectorParams) -> tuple[SpotBlob, ...]:
    """The configured spot stage: white-ringed EdgeDrawing spots, or HSV blobs as the fallback."""
    import cv2  # lazy: see the module docstring

    masks = frame_masks(cv2, frame, params)
    if params.spot_detector == "edge":
        found = edge_spot_blobs(cv2, frame, masks, params)
        if found is not None:
            return found.blobs
    return spot_blobs(cv2, masks, params)


def _partial(cluster: SpotCluster, width: int, height: int, params: DetectorParams) -> WristView:
    """A partial egg at the center of the cluster's search window (its core bbox grown by
    window_factor x d_med per side, clipped to the frame, as in the detector)."""
    grow = params.window_factor * cluster.d_med
    x, y, w, h = cluster.box
    left, top = max(0.0, x - grow), max(0.0, y - grow)
    right, bottom = min(float(width), x + w + grow), min(float(height), y + h + grow)
    return WristView(color=cluster.spots[0].color, partial=True, cx=(left + right) / 2 / width,
                     cy=(top + bottom) / 2 / height)


def egg_in_wrist_view(
    frame_bgr: Any, params: DetectorParams = WRIST_PARAMS, min_spots: int = WRIST_MIN_SPOT_CLUSTER
) -> WristView | None:
    """The egg in an HxWx3 uint8 BGR wrist frame: the largest full detection, else the spot cluster
    with the largest spot area that has at least `min_spots` spots (partial), else None."""
    frame = _frame(frame_bgr)
    eggs = detect_eggs(frame, params)
    if eggs:
        return WristView(color=eggs[0].color, partial=False, cx=eggs[0].cx, cy=eggs[0].cy)
    height, width = frame.shape[:2]
    clusters = cluster_spots(_spots(frame, params), params.spot_cluster_factor, params.core_spot_factor)
    best = next((cluster for cluster in clusters if len(cluster.spots) >= min_spots), None)
    return None if best is None else _partial(best, width, height, params)
