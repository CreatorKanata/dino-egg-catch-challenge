"""src/robot/vision/egg_masks.py: Color masks, spot blobs, and spot clusters for the egg detector.

First stage of egg_detector.py. From one BGR frame it builds the basket-free white-body mask and
one basket-free spot mask per color (egg_detector ORs the body with the spot colors of each
cluster, so a green egg never picks up red-looking basket pixels), finds the spot
blobs with their diameters, and groups them into clusters by single-linkage proximity scaled by
spot size. Each cluster is one egg hypothesis whose median spot diameter sets the scale of the
later segmentation (robot run 2026-09-24: tarp glints are not separable from the egg by color,
only by size). cluster_spots is pure; the rest takes the lazily imported cv2 module as argument.
"""

from collections.abc import Mapping
from dataclasses import dataclass, field
import math
from statistics import median
from typing import Any

import numpy as np

from robot.vision.config_vision import (
    BASKET_EXCLUDED_FROM_RED,
    BASKET_HSV,
    EGG_ASPECT_RANGE,
    EGG_BODY_HSV,
    EGG_BORDER_MARGIN_PX,
    EGG_CLOSE_KERNEL_PX,
    EGG_CORE_SPOT_FACTOR,
    EGG_EDGE_SPOT_MIN_FILL,
    EGG_ELLIPSE_FILL_RANGE,
    EGG_MIN_AREA_PX,
    EGG_MIN_SOLIDITY,
    EGG_MIN_SPOT_AREA_PX,
    EGG_MIN_SPOT_DIAMETER_PX,
    EGG_MIN_SPOT_FRACTION,
    EGG_MIN_SPOTS,
    EGG_OPEN_SPOT_FACTOR,
    EGG_REPAIR_KERNEL_FRACTION,
    EGG_SCALE_RANGE,
    EGG_SPOT_CLUSTER_FACTOR,
    EGG_SPOT_DETECTOR,
    EGG_SPOT_HSV,
    EGG_WINDOW_FACTOR,
)

HsvRange = tuple[tuple[int, int, int], tuple[int, int, int]]


@dataclass(frozen=True)
class DetectorParams:
    """All detector thresholds; the defaults come from config_vision.py."""

    body_hsv: HsvRange = EGG_BODY_HSV
    spot_hsv: Mapping[str, tuple[HsvRange, ...]] = field(default_factory=lambda: EGG_SPOT_HSV)
    basket_hsv: tuple[HsvRange, ...] = BASKET_HSV
    # Per spot color, the basket ranges to subtract instead of all of basket_hsv (red: only the
    # part that does not overlap the red spots).
    basket_for_spot: Mapping[str, tuple[HsvRange, ...]] = field(
        default_factory=lambda: {"red": BASKET_EXCLUDED_FROM_RED})
    spot_detector: str = EGG_SPOT_DETECTOR  # "hsv" or "edge"
    edge_spot_min_fill: float = EGG_EDGE_SPOT_MIN_FILL
    min_spot_area_px: int = EGG_MIN_SPOT_AREA_PX
    min_spot_diameter_px: float = EGG_MIN_SPOT_DIAMETER_PX
    spot_cluster_factor: float = EGG_SPOT_CLUSTER_FACTOR
    core_spot_factor: float = EGG_CORE_SPOT_FACTOR
    window_factor: float = EGG_WINDOW_FACTOR
    close_kernel_px: int = EGG_CLOSE_KERNEL_PX
    open_spot_factor: float = EGG_OPEN_SPOT_FACTOR
    repair_kernel_fraction: float = EGG_REPAIR_KERNEL_FRACTION
    min_area_px: int = EGG_MIN_AREA_PX
    aspect_range: tuple[float, float] = EGG_ASPECT_RANGE
    min_spots: int = EGG_MIN_SPOTS
    min_spot_fraction: float = EGG_MIN_SPOT_FRACTION
    scale_range: tuple[float, float] = EGG_SCALE_RANGE
    min_solidity: float = EGG_MIN_SOLIDITY
    ellipse_fill_range: tuple[float, float] = EGG_ELLIPSE_FILL_RANGE
    border_margin_px: int = EGG_BORDER_MARGIN_PX


DEFAULT_PARAMS = DetectorParams()


@dataclass(frozen=True)
class FrameMasks:
    """Basket-free masks of one frame: the white body and one spot mask per color (0/255)."""

    body: Any
    spots: Mapping[str, Any]

    @property
    def shape(self) -> tuple[int, int]:
        return self.body.shape[:2]


@dataclass(frozen=True)
class SpotBlob:
    """One spot: color, center, pixel area, equivalent diameter, and pixel bbox (x, y, w, h)."""

    color: str
    cx: float
    cy: float
    area: int
    diameter: float
    box: tuple[int, int, int, int]


@dataclass(frozen=True)
class SpotCluster:
    """Spots of one egg hypothesis; `core` are the large ones, which set the median diameter d_med
    and the joint bbox."""

    spots: tuple[SpotBlob, ...]
    core: tuple[SpotBlob, ...]
    d_med: float
    box: tuple[int, int, int, int]


def _in_ranges(cv2: Any, hsv: Any, ranges: tuple[HsvRange, ...]) -> Any:
    masks = [cv2.inRange(hsv, np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8)) for low, high in ranges]
    return np.bitwise_or.reduce(masks) if masks else np.zeros(hsv.shape[:2], dtype=np.uint8)


def _not_basket_for(cv2: Any, hsv: Any, color: str, not_basket: Any, params: DetectorParams) -> Any:
    ranges = params.basket_for_spot.get(color)
    return not_basket if ranges is None else cv2.bitwise_not(_in_ranges(cv2, hsv, ranges))


def frame_masks(cv2: Any, frame: Any, params: DetectorParams) -> FrameMasks:
    """HSV thresholds with the pink basket removed from the body and every spot mask (for red only
    the basket ranges that do not overlap the red spots)."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    not_basket = cv2.bitwise_not(_in_ranges(cv2, hsv, params.basket_hsv))
    spots = {color: _in_ranges(cv2, hsv, ranges) & _not_basket_for(cv2, hsv, color, not_basket, params)
             for color, ranges in params.spot_hsv.items()}
    return FrameMasks(body=_in_ranges(cv2, hsv, (params.body_hsv,)) & not_basket, spots=spots)


def spot_blobs(cv2: Any, masks: FrameMasks, params: DetectorParams) -> tuple[SpotBlob, ...]:
    """Every spot blob of at least min_spot_area_px, per color."""
    blobs = []
    for color, mask in masks.spots.items():
        count, _, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        for index in range(1, count):
            area = int(stats[index, cv2.CC_STAT_AREA])
            if area >= params.min_spot_area_px:
                x, y, w, h = (int(value) for value in stats[index, :4])
                blobs.append(SpotBlob(color, float(centroids[index][0]), float(centroids[index][1]), area,
                                      2 * math.sqrt(area / math.pi), (x, y, w, h)))
    return tuple(blobs)


def _joint_box(spots: tuple[SpotBlob, ...]) -> tuple[int, int, int, int]:
    left = min(spot.box[0] for spot in spots)
    top = min(spot.box[1] for spot in spots)
    right = max(spot.box[0] + spot.box[2] for spot in spots)
    bottom = max(spot.box[1] + spot.box[3] for spot in spots)
    return left, top, right - left, bottom - top


def _cluster(spots: tuple[SpotBlob, ...], core_factor: float) -> SpotCluster:
    largest = max(spot.diameter for spot in spots)
    core = tuple(spot for spot in spots if spot.diameter >= core_factor * largest)
    return SpotCluster(spots, core, median(spot.diameter for spot in core), _joint_box(core))


def cluster_spots(
    blobs: tuple[SpotBlob, ...], factor: float = EGG_SPOT_CLUSTER_FACTOR, core_factor: float = EGG_CORE_SPOT_FACTOR
) -> tuple[SpotCluster, ...]:
    """Single-linkage clusters: two spots of the same color link when their centers are closer
    than factor x the larger diameter (an egg has one spot color, so red-looking basket pixels
    never join a green egg). Pure; largest total spot area first."""
    parent = list(range(len(blobs)))

    def root(index: int) -> int:
        while parent[index] != index:
            index = parent[index]
        return index

    for i, first in enumerate(blobs):
        for j in range(i + 1, len(blobs)):
            second = blobs[j]
            reach = factor * max(first.diameter, second.diameter)
            near = math.dist((first.cx, first.cy), (second.cx, second.cy)) < reach
            if near and first.color == second.color:
                parent[root(j)] = root(i)
    groups: dict[int, tuple[SpotBlob, ...]] = {}
    for index, blob in enumerate(blobs):
        groups = {**groups, root(index): groups.get(root(index), ()) + (blob,)}
    clusters = (_cluster(spots, core_factor) for spots in groups.values())
    return tuple(sorted(clusters, key=lambda cluster: sum(spot.area for spot in cluster.spots), reverse=True))
