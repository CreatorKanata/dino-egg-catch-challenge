"""src/robot/vision/egg_refine.py: Keep a white background out of an egg component (fence, cap).

Split out of egg_detector.py (300-line limit). A white wall, a white PVC pipe, or clothes behind
the field touch the egg and pass the body mask (robot run 2026-09-25, captures 20260925-010137 and
-010212), so color cannot separate them. Two geometric steps can:
- the edge fence: Canny edges of the window (V channel), dilated, are removed from the mask after
  the crack-bridging close, so the egg's shaded silhouette separates it from a flat white wall and
  from the pipe. The fence is not applied inside the convex hull of the core spots' pixels, so the
  crack and the spot borders do not cut the egg apart; the silhouette lies outside or on that hull
  and stays fenced (sparing a band around each spot instead opened the fence to the wall where a
  spot sits on the silhouette). The component is chosen by spot votes (center or ring). The thin
  rim beyond a spot on the silhouette is fenced off, so the bbox can be a few px narrower;
- the hull cap: the component is intersected with the convex hull of the core and missed spots
  grown by EGG_HULL_CAP_FACTOR x d, so a white background never extends it beyond a plausible egg.
Also the missed-spot completion (color inside the hull, spot-sized blobs touching it).
"""

import math
from typing import Any

import numpy as np

from robot.vision.egg_masks import DetectorParams, FrameMasks, SpotCluster


def fence(cv2: Any, masks: FrameMasks, window: tuple[int, int, int, int], params: DetectorParams) -> Any:
    """Boolean window mask of edge-fence pixels (Canny on the V channel, dilated)."""
    wx, wy, ww, wh = window
    value = np.ascontiguousarray(masks.hsv[wy:wy + wh, wx:wx + ww, 2])
    edges = cv2.Canny(value, params.fence_canny[0], params.fence_canny[1])
    grow = 2 * params.fence_dilate_px + 1
    return cv2.dilate(edges, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (grow, grow))) > 0


def ring_vote(labels: Any, areas: Any, cluster: SpotCluster, origin: tuple[int, int], scale: int,
              reach_px: float, samples: int = 32) -> int:
    """The label touched by the most core spots (center or surrounding ring; ties: larger area),
    0 if none. The ring, `reach_px` outside each spot's half size, still finds the body where the
    fence cuts a spot loose on the silhouette."""
    rows, cols = labels.shape
    tally: dict[int, int] = {}
    for spot in cluster.core:
        radius = (max(spot.box[2], spot.box[3]) / 2 + reach_px) / scale
        cx, cy = (spot.cx - origin[0]) / scale, (spot.cy - origin[1]) / scale
        points = [(cx, cy)] + [(cx + radius * math.cos(2 * math.pi * i / samples),
                                cy + radius * math.sin(2 * math.pi * i / samples)) for i in range(samples)]
        touched = {int(labels[min(rows - 1, max(0, int(y))), min(cols - 1, max(0, int(x)))]) for x, y in points}
        tally = {**tally, **{label: tally.get(label, 0) + 1 for label in touched if label}}
    if not tally:
        return 0
    return max(tally, key=lambda label: (tally[label], areas[label]))


def regrow(cv2: Any, component: Any, unfenced: Any, params: DetectorParams) -> Any:
    """Give back the egg's own half of the fence band: grow the component by fence_dilate_px + 1
    within the unfenced mask. That stops short of the band's far half, so it never reaches a wall
    on the other side of the edge; without it every egg would lose ~3 px at its silhouette."""
    grow = 2 * (params.fence_dilate_px + 1) + 1
    grown = cv2.dilate(component.astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (grow, grow)))
    return (grown > 0) & (unfenced > 0)


def core_spot_pixels(spots: Any, cluster: SpotCluster, origin: tuple[int, int]) -> Any:
    """Boolean window mask: the spot pixels inside the cluster's core spot boxes."""
    keep = np.zeros(spots.shape, dtype=bool)
    for x, y, w, h in (spot.box for spot in cluster.core):
        left, top = max(0, x - origin[0]), max(0, y - origin[1])
        keep[top:y - origin[1] + h, left:x - origin[0] + w] = True
    return keep & (spots > 0)


def spot_hull(cv2: Any, spot_pixels: Any) -> Any:
    """Boolean window mask: the convex hull of the core spots' pixels (not their boxes, whose corners
    reach past the silhouette). The fence is not applied inside it, so the crack and the spot
    borders do not cut the egg, while the silhouette, outside or on the hull, stays fenced."""
    hull = np.zeros(spot_pixels.shape, dtype=np.uint8)
    outlines, _ = cv2.findContours(spot_pixels.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if outlines:
        cv2.fillConvexPoly(hull, cv2.convexHull(np.concatenate(outlines)), 1)
    return hull > 0


def hull_cap(cv2: Any, cluster: SpotCluster, window: tuple[int, int, int, int], extra: Any,
             params: DetectorParams, factor: float | None = None) -> Any:
    """Boolean window mask: the convex hull of the core spots and of `extra` (the missed spots, spot
    color only, so never wall) grown by hull_cap_factor x d_med.

    Growing a convex polygon by r equals filling it and drawing its outline 2r thick (round joins),
    far cheaper than a dilation with a ~2r kernel.
    """
    wx, wy, ww, wh = window
    corners = [(x - wx + dx, y - wy + dy) for spot in cluster.core for x, y, w, h in (spot.box,)
               for dx, dy in ((0, 0), (w, 0), (0, h), (w, h))]
    points = [np.array(corners, dtype=np.int32).reshape(-1, 1, 2)]
    if extra is not None:
        points += list(cv2.findContours(extra.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0])
    hull = cv2.convexHull(np.concatenate(points))
    cap = np.zeros((wh, ww), dtype=np.uint8)
    cv2.fillConvexPoly(cap, hull, 1)
    if params is not None:
        grow = params.hull_cap_factor if factor is None else factor
        thickness = max(1, int(round(2 * grow * cluster.d_med)))
        cv2.polylines(cap, [hull], True, 1, thickness=thickness, lineType=cv2.LINE_8)
    return cap > 0


def missed_spots(cv2: Any, masks: FrameMasks, component: Any, cluster: SpotCluster,
                 window: tuple[int, int, int, int], params: DetectorParams) -> Any:
    """Spots the spot stage missed: HSV pixels of the cluster's color inside the component's convex
    hull, plus whole spot-sized, compact color blobs touching the hull (spots on the egg's edge,
    which the hull chord would cut). Tarp of the same color is larger than a spot and stays out."""
    wx, wy, ww, wh = window
    outlines, _ = cv2.findContours(component.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    hull = np.zeros((wh, ww), dtype=np.uint8)
    if outlines:
        cv2.fillConvexPoly(hull, cv2.convexHull(np.concatenate(outlines)), 1)
    colors = sorted({spot.color for spot in cluster.core})
    color = masks.colors[colors[0]][wy:wy + wh, wx:wx + ww] > 0  # one spot color per cluster
    count, labels, stats, _ = cv2.connectedComponentsWithStats(color.astype(np.uint8), connectivity=8)
    max_area = params.spot_blob_max_factor * math.pi / 4 * cluster.d_med ** 2
    touching = np.unique(labels[(hull > 0) & color])
    small = [label for label in touching if label and stats[label, cv2.CC_STAT_AREA] <= max_area
             and stats[label, cv2.CC_STAT_AREA] >= params.spot_blob_min_extent
             * stats[label, cv2.CC_STAT_WIDTH] * stats[label, cv2.CC_STAT_HEIGHT]]
    keep = np.zeros(count, dtype=bool)
    keep[small] = True
    return (color & (hull > 0)) | keep[labels]


def box_of(mask: Any, origin: tuple[int, int]) -> tuple[int, int, int, int]:
    """Frame bbox (x, y, w, h) of a boolean window mask; zeros when empty."""
    rows, cols = np.nonzero(mask)
    if rows.size == 0:
        return 0, 0, 0, 0
    return (int(cols.min()) + origin[0], int(rows.min()) + origin[1],
            int(cols.max() - cols.min()) + 1, int(rows.max() - rows.min()) + 1)
