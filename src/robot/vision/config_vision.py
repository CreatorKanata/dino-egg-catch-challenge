"""src/robot/vision/config_vision.py: Tunables of the front-camera egg and basket detectors.

Split out of robot/config.py (which would pass 300 lines otherwise); config.py still holds every
other runtime value, including the Auto Catch alignment target and size precondition. The Auto
Release basket detector, its size precondition, and the basket alignment target and tolerances are
at the end of this file. Stdlib-only.
All values are placeholders: calibrate at the venue with `c` captures and
`python -m robot.vision.inspect --debug`. Checked against the robot's front captures
20260924-220742 (best position), -220336 (pink basket behind the egg), -220404 (no basket),
-222325 (far egg, bluish lower half), -223853 (strong light, glints on the tarp wrinkles), the
red-spotted egg -225648 (best position, room light), -225701 (extra light), -230143 (pink basket
directly behind it), -230437, and -232411 (green egg in front, far red egg at the edge).
Egg colors (owner decisions): green, red, orange; blue-spotted eggs were dropped on 2026-09-24
(their captures -231156/-231202 remain a regression: no egg of another color may be reported).
HSV uses OpenCV ranges: H 0-179, S and V 0-255.
"""

from typing import Final

from robot.config import ALIGN_FULL_SPEED_ERROR_CX, ALIGN_FULL_SPEED_ERROR_H, ALIGN_MAX_XY, ALIGN_MIN_XY

# Body: white incl. the shadowed half (V >= 40), as ranges. S <= 50 at any hue; warm whites (H 0-40)
# up to S 65, because the blue egg's white (H 19-30) reaches S 61-65 (p95, -231156) while bright tarp
# reflections (bluish, S 50-65) must stay out (they joined the far egg of -222325 at S <= 65).
EGG_BODY_HSV: Final = (((0, 0, 40), (179, 50, 255)), ((0, 0, 40), (40, 65, 255)))
EGG_SPOT_HSV: Final = {
    # Green spot hue medians 76-82 on the five green captures; H 90 is the old boundary to the
    # dropped blue class (blue spot medians were 99-109).
    "green": (((35, 60, 15), (90, 255, 255)),),
    # Red spots (-225648/-225701/-230143/-230437): S 157-204 (p5-p95), V down to 33 on the shadowed
    # center spot; hue mostly 172-179, and the low end never above H 4 (p99), so red ends at H 7.
    "red": (((0, 140, 25), (7, 255, 255)), ((174, 140, 25), (179, 255, 255))),
    # Orange (painted egg, owner decision 2026-09-24): PLACEHOLDER until the owner captures it;
    # starts above red's H 7.
    "orange": (((8, 120, 60), (25, 255, 255)),),
}
# Pink basket (-220336: H p5/50/95 135/172/176, S 77/131/182, V 21/64/73). Two ranges so it never
# overlaps the red spots: H 150-173 at any S >= 60, and H 174-179 only below the red S floor (140).
# Excluded from the body and the green/orange spot masks; the red mask drops only the first range
# (the second is below its S floor anyway). A few basket pixels at H 174-176 with S up to ~182 can
# still reach the red mask; the cluster, scale, and spot-fraction rules reject them. Auto Release
# uses the looser BASKET_DETECT_HSV below for its basket detector.
BASKET_HSV: Final = (((150, 60, 20), (173, 255, 255)), ((174, 60, 20), (179, 139, 255)))
BASKET_EXCLUDED_FROM_RED: Final = BASKET_HSV[:1]

# Segmentation is anchored on the spots and scaled by their size (robot run 2026-09-24: under a
# strong light, white glints on the tarp wrinkles are not separable from the egg by color).
# Spot stage: "edge" (default; OpenCV contrib EdgeDrawing ellipses with a white ring around them
# and one spot color inside, needs opencv-contrib-python-headless) or "hsv" (connected components
# of the spot color masks; also the fallback without cv2.ximgproc). "hsv" cannot find blue spots on
# the blue tarp: blue-spotted eggs (dropped 2026-09-24) motivated "edge", which stays the default
# because the white-ring test also makes spot detection robust to lighting and to tarp glints.
EGG_SPOT_DETECTOR: Final = "edge"
EGG_EDGE_SPOT_MIN_FILL: Final = 0.5  # "edge": majority of the interior in one spot color
# "edge": egg white around a spot, the body ranges with V >= 60.
EGG_RING_WHITE_HSV: Final = tuple(((low[0], low[1], 60), high) for low, high in EGG_BODY_HSV)
EGG_RING_SCALES: Final = (1.15, 1.6)  # "edge": the ring spans these multiples of the ellipse axes
# "edge": share of in-frame ring pixels that must be white; true spots measured 0.48-0.89 (a spot
# at the egg's edge or next to another spot has less white around it), tarp patches near 0.
EGG_RING_WHITE_FRACTION: Final = 0.35
EGG_MIN_SPOT_AREA_PX: Final = 40  # spot blobs smaller than this are ignored
# Clusters whose median core spot is narrower than this are skipped before segmentation (speed):
# with egg height ~4 spot diameters that is an egg under ~40 px (0.08 of 480), below the "too far" limit.
EGG_MIN_SPOT_DIAMETER_PX: Final = 10.0
# Spots link into one egg hypothesis when their centers are closer than this x the larger spot
# diameter (d = 2 * sqrt(area / pi)); single linkage.
# 4.0: the edge stage often finds only the outer spots, up to ~4.5 d apart on an egg ~5.5 d wide
# (blue egg -231156: two spots 240 px apart at d 67-70).
EGG_SPOT_CLUSTER_FACTOR: Final = 4.0
# A cluster's scale, window, and component vote use only its core spots, those at least this x the
# largest spot diameter in the cluster; smaller greenish blobs (dark tarp folds) may join but do not
# shrink the scale. 0.35 (not 0.5) keeps the red egg's one oversized center spot (d 140 vs 34-79)
# from inflating the median: scales are then 3.25-4.1 on all eight check captures.
EGG_CORE_SPOT_FACTOR: Final = 0.35
EGG_WINDOW_FACTOR: Final = 2.0  # search window = cluster bbox grown by this x median d per side
EGG_CLOSE_KERNEL_PX: Final = 15  # joins the body across thin dark lines (the shell's crack)
EGG_OPEN_SPOT_FACTOR: Final = 0.5  # then an open of this x median d removes glints thinner than that
# Spot-colored pixels inside the egg's convex hull join it (spots the spot stage missed); whole
# spot-colored blobs touching the egg join too when no larger than this x a spot's area (pi/4 d^2),
# so spots on the egg's edge are not cut by the hull, while background of the spot's color stays out.
EGG_SPOT_BLOB_MAX_FACTOR: Final = 3.0
EGG_SPOT_BLOB_MIN_EXTENT: Final = 0.45  # and compact: blob area / bbox area (a disk 0.79, thin arcs far less)
# Shape test on the outline closed with max(EGG_CLOSE_KERNEL_PX, fraction * min(w, h)) px (repairs
# bites from tarp reflections, -222325).
EGG_REPAIR_KERNEL_FRACTION: Final = 0.20
EGG_MIN_AREA_PX: Final = 1500
EGG_ASPECT_RANGE: Final = (0.5, 2.2)  # bbox width / height
EGG_MIN_SPOTS: Final = 1
# Spot pixels / egg area: 0.25-0.37 on every check capture, 0.07 for a false white patch with two
# small spots at the left edge of -222325.
EGG_MIN_SPOT_FRACTION: Final = 0.12
# Egg bbox height / median core spot diameter: 3.1-4.3 for the green egg (and the dropped blue one), 2.46-2.86 for the
# red egg (the edge stage finds only its two big spots); 1.8 keeps margin below the red egg.
EGG_SCALE_RANGE: Final = (1.8, 6.5)
EGG_MIN_SOLIDITY: Final = 0.85  # contour area / convex hull area: an egg outline is convex
# Contour area / fitted-ellipse area; only for eggs clear of the frame border (a partly visible
# egg is a truncated ellipse, still convex, so it keeps only the solidity test).
EGG_ELLIPSE_FILL_RANGE: Final = (0.75, 1.25)
EGG_BORDER_MARGIN_PX: Final = 2  # bbox closer than this to an edge = touches the border

if EGG_SPOT_DETECTOR not in ("hsv", "edge"):
    raise ValueError('EGG_SPOT_DETECTOR must be "hsv" or "edge"')
if not (0 < EGG_CORE_SPOT_FACTOR <= 1 and 0 < EGG_EDGE_SPOT_MIN_FILL <= 1 and 0 < EGG_MIN_SPOT_FRACTION < 1):
    raise ValueError("EGG_CORE_SPOT_FACTOR, EGG_EDGE_SPOT_MIN_FILL, and EGG_MIN_SPOT_FRACTION must be in (0, 1]")
if not (0 < EGG_SCALE_RANGE[0] < EGG_SCALE_RANGE[1] and EGG_SPOT_CLUSTER_FACTOR > 0 and EGG_WINDOW_FACTOR > 0):
    raise ValueError("Need 0 < EGG_SCALE_RANGE low < high and positive cluster and window factors")

# --- Pink basket detector and alignment target (Auto Release, Phase 2) ------------------------
# Looser than BASKET_HSV (egg exclusion, unchanged): at the release position the basket is dark
# (V 40-52) and desaturated (S median 49-77), measured on captures -232927 and -232953.
BASKET_DETECT_HSV: Final = ((158, 30, 18), (179, 200, 255))
BASKET_OPEN_PX: Final = 11  # full-resolution kernels: the open removes red egg spots (~1-2 % of the frame)
BASKET_CLOSE_PX: Final = 25
BASKET_MORPH_SCALE: Final = 2  # morphology runs on a mask downscaled by this factor (6 ms -> ~1.3 ms)
BASKET_MIN_AREA_FRACTION: Final = 0.05  # largest pink component / frame area
BASKET_MIN_FILL: Final = 0.35  # component area / its bbox area
AUTO_RELEASE_MIN_BASKET_W: Final = 0.30  # below: "Basket too far" (the basket is never too close)
# Basket bbox at the release position (-232927 room light, -232953 extra light: cx 0.52-0.54,
# cy 0.40-0.42, w 0.78-0.83, h 0.68-0.73). The controller uses cx and w; cy and h only draw the box.
RELEASE_TARGET_CX: Final = 0.53
RELEASE_TARGET_CY: Final = 0.41
RELEASE_TARGET_W: Final = 0.80
RELEASE_TARGET_H: Final = 0.70
RELEASE_TOL_CX: Final = 0.05
RELEASE_TOL_W_FAR: Final = 0.04  # the basket may be at most this much narrower (farther) than the target
RELEASE_TOL_W_NEAR: Final = 0.10  # and at most this much wider (closer)

if not (0 < BASKET_MIN_AREA_FRACTION < 1 and 0 < BASKET_MIN_FILL <= 1 and BASKET_MORPH_SCALE >= 1):
    raise ValueError("Need 0 < BASKET_MIN_AREA_FRACTION < 1, 0 < BASKET_MIN_FILL <= 1, BASKET_MORPH_SCALE >= 1")
if not (RELEASE_TOL_CX > 0 and RELEASE_TOL_W_FAR > 0 and RELEASE_TOL_W_NEAR > 0):
    raise ValueError("RELEASE_TOL_CX, RELEASE_TOL_W_FAR, and RELEASE_TOL_W_NEAR must be positive")
if not 0 < AUTO_RELEASE_MIN_BASKET_W < RELEASE_TARGET_W - RELEASE_TOL_W_FAR:
    raise ValueError("Need 0 < AUTO_RELEASE_MIN_BASKET_W < RELEASE_TARGET_W - RELEASE_TOL_W_FAR")
if not (RELEASE_TOL_CX < ALIGN_FULL_SPEED_ERROR_CX and RELEASE_TOL_W_FAR < ALIGN_FULL_SPEED_ERROR_H):
    raise ValueError("Full-speed errors must exceed RELEASE_TOL_CX and RELEASE_TOL_W_FAR")
# Same stall guard as the egg path (config.py): the command just outside every edge must reach ALIGN_MIN_XY.
_EDGES = ((RELEASE_TOL_CX, ALIGN_FULL_SPEED_ERROR_CX), (RELEASE_TOL_W_FAR, ALIGN_FULL_SPEED_ERROR_H),
          (RELEASE_TOL_W_NEAR, ALIGN_FULL_SPEED_ERROR_H))
if any(ALIGN_MAX_XY * min(1.0, tol / full) < ALIGN_MIN_XY - 1e-9 for tol, full in _EDGES):
    raise ValueError("Need ALIGN_MAX_XY * min(1, release tolerance / full-speed error) >= ALIGN_MIN_XY")
