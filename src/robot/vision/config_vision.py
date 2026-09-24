"""src/robot/vision/config_vision.py: Tunables of the front-camera egg detector (Auto Catch).

Split out of robot/config.py (which would pass 300 lines otherwise); config.py still holds every
other runtime value, including the alignment target and the size precondition. Stdlib-only.
All values are placeholders: calibrate at the venue with `c` captures and
`python -m robot.vision.inspect --debug`. Checked against the robot's front captures
20260924-220742 (best position), -220336 (pink basket behind the egg), -220404 (no basket),
-222325 (far egg, bluish lower half), and -223853 (strong light, glints on the tarp wrinkles).
HSV uses OpenCV ranges: H 0-179, S and V 0-255.
"""

from typing import Final

# Body: white incl. the shadowed half (V >= 40); S <= 50 keeps the white pipe at the left out.
EGG_BODY_HSV: Final = ((0, 0, 40), (179, 50, 255))
EGG_SPOT_HSV: Final = {
    "green": (((35, 60, 15), (100, 255, 255)),),  # teal-looking spots (H up to 100), shadowed down to V 15
    "blue": (((101, 120, 90), (130, 255, 255)),),  # S/V floor keeps the blue tarp (S 90-150, V < 90) out
    "red": (((0, 150, 60), (10, 255, 255)), ((170, 150, 60), (179, 255, 255))),  # S >= 150: not the pink basket
}
# Pink basket (measured on the robot: H 165-179, S 90-170, V 48-68). Excluded from the body and spot
# masks so the basket never joins an egg or adds spots. Auto Release will reuse it for its detector.
BASKET_HSV: Final = ((150, 60, 40), (179, 200, 255))

# Segmentation is anchored on the spots and scaled by their size (robot run 2026-09-24: under a
# strong light, white glints on the tarp wrinkles are not separable from the egg by color).
# Spot stage: "hsv" (connected components of the spot color masks) or "edge" (OpenCV contrib
# EdgeDrawing ellipses verified by the spot color inside; needs opencv-contrib-python-headless,
# falls back to "hsv" without cv2.ximgproc). On the check captures (2026-09-24) "hsv" found one
# egg in all seven test images at 8-11 ms; "edge" found fewer spots (2-3 per egg), matched on the
# five robot captures but split the old front_ref screenshot into two half eggs, at 10-13 ms.
EGG_SPOT_DETECTOR: Final = "hsv"
EGG_EDGE_SPOT_MIN_FILL: Final = 0.6  # "edge": share of the ellipse area in one spot color
EGG_MIN_SPOT_AREA_PX: Final = 40  # spot blobs smaller than this are ignored
# Clusters whose median core spot is narrower than this are skipped before segmentation (speed):
# with egg height ~4 spot diameters that is an egg under ~40 px (0.08 of 480), below the "too far" limit.
EGG_MIN_SPOT_DIAMETER_PX: Final = 10.0
# Spots link into one egg hypothesis when their centers are closer than this x the larger spot
# diameter (d = 2 * sqrt(area / pi)); single linkage.
EGG_SPOT_CLUSTER_FACTOR: Final = 3.0
# A cluster's scale, window, and component vote use only its core spots, those at least this x the
# largest spot diameter in the cluster; smaller greenish blobs (dark tarp folds) may join but do not
# shrink the scale.
EGG_CORE_SPOT_FACTOR: Final = 0.5
EGG_WINDOW_FACTOR: Final = 2.0  # search window = cluster bbox grown by this x median d per side
EGG_CLOSE_KERNEL_PX: Final = 15  # joins the body across thin dark lines (the shell's crack)
EGG_OPEN_SPOT_FACTOR: Final = 0.5  # then an open of this x median d removes glints thinner than that
# Shape test on the outline closed with max(EGG_CLOSE_KERNEL_PX, fraction * min(w, h)) px (repairs
# bites from tarp reflections, -222325).
EGG_REPAIR_KERNEL_FRACTION: Final = 0.20
EGG_MIN_AREA_PX: Final = 1500
EGG_ASPECT_RANGE: Final = (0.5, 2.2)  # bbox width / height
EGG_MIN_SPOTS: Final = 1
# Spot pixels / egg area: 0.25-0.37 on every check capture, 0.07 for a false white patch with two
# small spots at the left edge of -222325.
EGG_MIN_SPOT_FRACTION: Final = 0.12
# Egg bbox height / median spot diameter; measured ~4 on the captures (spot / egg height ~0.25).
EGG_SCALE_RANGE: Final = (2.5, 6.0)
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
