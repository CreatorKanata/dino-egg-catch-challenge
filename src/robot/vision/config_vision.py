"""src/robot/vision/config_vision.py: Tunables of the front-camera egg and basket detectors.

Split out of robot/config.py (which would pass 300 lines otherwise); config.py still holds every
other runtime value, including the Auto Catch alignment target and size precondition. The Auto
Release basket detector, its size precondition, and the basket alignment target and tolerances
follow, then the Auto Catch wrist-view check at the end of this file. Stdlib-only.
All values are placeholders: calibrate at the venue with `c` captures and
`python -m robot.vision.inspect --debug`. Checked against the robot's front captures
20260924-220742 (best position), -220336 (pink basket behind the egg), -220404 (no basket),
-222325 (far egg, bluish lower half), -223853 (strong light, glints on the tarp wrinkles), the
red-spotted egg -225648 (best position, room light), -225701 (extra light), -230143 (pink basket
directly behind it), -230437, -232411 (green egg in front, far red egg at the edge), 20260925-003436,
and 20260925-010137/-010212 (red egg at the best position with a white wall, outlets, and a white
PVC pipe directly behind it); basket-only captures 20260924-232927/-232953/-234124 give no egg.
Egg colors (owner decisions): green, red, yellow (owner's name, orange-looking paint; replaces
"orange", 2026-09-25); blue-spotted eggs were dropped on 2026-09-24
(their captures -231156/-231202 remain a regression: no egg of another color may be reported).
HSV uses OpenCV ranges: H 0-179, S and V 0-255.
"""

from typing import Final

from robot.config import ALIGN_FULL_SPEED_ERROR_CX, ALIGN_FULL_SPEED_ERROR_H, ALIGN_MAX_XY, ALIGN_MIN_XY

# Body: white incl. the shadowed half (V >= 40), as ranges. S <= 50 at any hue; warm whites (H 0-40)
# up to S 65, because the blue egg's white (H 19-30) reaches S 61-65 (p95, -231156) while bright tarp
# reflections (bluish, S 50-65) must stay out (they joined the far egg of -222325 at S <= 65).
# Third range: the egg white lit bluish by the tarp (yellow egg -224252: H 96-107, S 61-158,
# V 89-161), bright enough (V >= 85) to stay apart from the tarp itself (S 160-190, V 30-73).
EGG_BODY_HSV: Final = (
    ((0, 0, 40), (179, 50, 255)), ((0, 0, 40), (40, 65, 255)), ((85, 0, 85), (125, 150, 255)),
)
EGG_SPOT_HSV: Final = {
    # Green spot hue medians 76-82 on the five green captures; H 90 is the old boundary to the
    # dropped blue class (blue spot medians were 99-109).
    "green": (((35, 60, 15), (90, 255, 255)),),
    # Red spots: S 157-204 (p5-p95) under the earlier light, but down to S 95-118 (p10) and H 172 in
    # -010137/-010212; V down to 26 on the shadowed center spot. The low end never exceeds H 4 (p99),
    # so red ends at H 7.
    "red": (((0, 100, 25), (7, 255, 255)), ((170, 100, 25), (179, 255, 255))),
    # Yellow (owner's name; the painted, glossy spots look orange to the camera): PLACEHOLDER from
    # two captures, 20260925-223516 (spots H 1-22, S 57-180, V 35-177; glossy highlights down to
    # S 38) and -224252 (more light: spot S p50 81-129). Overlaps red at H 5-7: each spot takes the
    # color with the larger interior share (ties: red, listed first) and a cluster has one color, so
    # a red egg stays red (tests). Recalibrate with more yellow captures (light on/off, far, basket).
    # H <= 18: warm egg white (H 19-30, S up to 65, -231202) and beige clutter (H ~22) stay out.
    "yellow": (((5, 70, 30), (18, 255, 255)),),
}
# Pink basket (-220336: H p5/50/95 135/172/176, S 77/131/182, V 21/64/73). Two ranges so it never
# overlaps the red spots: H 150-169 at any S >= 60, and H 170-179 only below the red S floor (100).
# Basket and red spots now share H 170-176 at S 100-182; the white-ring spot test, same-color
# clusters, and the cap keep basket pixels out of eggs (basket-only captures -232927/-232953/-234124
# give no egg).
# Excluded from the body and the green/yellow spot masks; the red mask drops only the first range
# (the second is below its S floor anyway). Auto Release
# uses the looser BASKET_DETECT_HSV below for its basket detector.
BASKET_HSV: Final = (((150, 60, 20), (169, 255, 255)), ((170, 60, 20), (179, 99, 255)))
BASKET_EXCLUDED_FROM_RED: Final = BASKET_HSV[:1]

# Segmentation is anchored on the spots and scaled by their size (robot run 2026-09-24: under a
# strong light, white glints on the tarp wrinkles are not separable from the egg by color).
# Spot stage: "edge" (default; OpenCV contrib EdgeDrawing ellipses with a white ring around them
# and one spot color inside, needs opencv-contrib-python-headless) or "hsv" (connected components
# of the spot color masks; also the fallback without cv2.ximgproc). "hsv" cannot find blue spots on
# the blue tarp: blue-spotted eggs (dropped 2026-09-24) motivated "edge", which stays the default
# because the white-ring test also makes spot detection robust to lighting and to tarp glints.
EGG_SPOT_DETECTOR: Final = "edge"
# Glossy paint punches low-saturation holes into spots (yellow egg); each spot mask is closed with
# this kernel before any fill test.
EGG_SPOT_CLOSE_PX: Final = 5
# EdgeDrawing misses spots from frame to frame (glossy yellow egg: 1-3 of its spots under sensor
# noise), so the edge stage also accepts HSV color blobs, ellipse-fitted, with this axis ratio at
# most, passing the same white-ring and fill tests, and not already covered by an EdgeDrawing spot.
EGG_BLOB_SPOT_MAX_ASPECT: Final = 2.5
# ... and a minor axis of at least this (px): an egg with smaller spots (~4 d tall) is below the
# "too far" size anyway, and tiny colored specks (an outlet LED, -010137) must not become spots.
EGG_BLOB_SPOT_MIN_PX: Final = 18.0
EGG_EDGE_SPOT_MIN_FILL: Final = 0.5  # "edge": majority of the interior in one spot color
# "edge": egg white around a spot, the body ranges with V >= 60 (the shaded range keeps V >= 85).
EGG_RING_WHITE_HSV: Final = tuple(((low[0], low[1], max(60, low[2])), high) for low, high in EGG_BODY_HSV)
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
# White backgrounds (wall, PVC pipe, clothes; -010137/-010212) touch the egg and pass the body mask.
# Edge fence: Canny (low, high) edges of the window's V channel, dilated by EGG_FENCE_DILATE_PX, are
# removed after the crack close; the egg body is then picked by spot ring votes (egg_refine.py).
EGG_FENCE_CANNY: Final = (40, 120)
EGG_FENCE_DILATE_PX: Final = 3
# Hull cap: the component is cut to the core spots' convex hull grown by this x d. On all check
# captures the egg bbox reaches at most 1.02 d beyond its spots' bbox, so 1.4 leaves margin.
EGG_HULL_CAP_FACTOR: Final = 1.4
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
EGG_MIN_SOLIDITY: Final = 0.80  # contour area / convex hull area: an egg outline is convex. Real eggs measure 0.95-0.98; a shadowed red egg (capture 20260925-003436) hit 0.85, so the floor is 0.80 (glints are already removed by the spot windows)
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

# --- Wrist-view egg check at the catch pose (Auto Catch, Phase 3 step 1; robot/vision/wrist_check.py) --
# Disabled until verified: enable after the owner captures wrist frames at the catch pose (`c` saves
# them) and `python -m robot.vision.inspect captures/<stamp>-wrist.png --wrist` confirms detections.
WRIST_CHECK_ENABLED: Final = False
WRIST_CHECK_FRAMES: Final = 10  # frames held at the catch pose; an egg in any of them passes the check
# Close-up mode (captures 20260925-014442/-014418/-014506): the egg fills the view and is cut by the
# border, so no egg-shape rules apply. The egg is present when at least WRIST_MIN_SPOTS spots
# (EdgeDrawing ellipse or HSV blob, minor axis >= WRIST_MIN_SPOT_PX, mostly one egg color inside)
# have egg white on at least WRIST_RING_WHITE_FRACTION of the in-frame part of their ring, and at
# least WRIST_RING_MIN_INSIDE of the ring is inside the frame. The white gripper parts have no spots;
# the pink basket edge has no white ring.
WRIST_MIN_SPOTS: Final = 1
WRIST_MIN_SPOT_PX: Final = 40
WRIST_RING_WHITE_FRACTION: Final = 0.5
WRIST_RING_MIN_INSIDE: Final = 0.25
# Spots are disks (axis ratio 1.1-1.3 on the near capture) filled with one color (share 0.96-1.00);
# the teal gripper part (ratio 1.7) and the pink basket edge (ratio 2.4, red share ~0.5) fail these.
WRIST_MAX_SPOT_ASPECT: Final = 1.5
WRIST_SPOT_MIN_FILL: Final = 0.7
if not (WRIST_CHECK_FRAMES >= 1 and WRIST_MIN_SPOTS >= 1 and WRIST_MIN_SPOT_PX > 0):
    raise ValueError("WRIST_CHECK_FRAMES and WRIST_MIN_SPOTS must be >= 1 and WRIST_MIN_SPOT_PX positive")
if not (0 < WRIST_RING_WHITE_FRACTION <= 1 and 0 < WRIST_RING_MIN_INSIDE <= 1):
    raise ValueError("WRIST_RING_WHITE_FRACTION and WRIST_RING_MIN_INSIDE must be in (0, 1]")
