"""src/robot/vision/inspect.py: Offline egg-detector check (`python -m robot.vision.inspect img.png`).

Runs the detector with the thresholds in config.py on one image (normally a capture from the
signboard's `c` key), prints every detection, and writes `<image>-detected.png` with the boxes
and the alignment target drawn, so the owner can tune the HSV ranges at the venue without the
robot. `--debug` also lists every spot cluster (core spots, median spot diameter, search window)
with its egg measurements and the rule that rejected it. `--rgb` reads an RGB-ordered image, such as a
signboard screenshot of the Pi cameras taken before the color-order fix (PI_CAMERA_COLOR_ORDER in
config.py).
"""

import argparse
from pathlib import Path
import sys
from typing import Any

import cv2

from robot.config import ALIGN_TARGET_CX, ALIGN_TARGET_CY, ALIGN_TARGET_H, ALIGN_TARGET_W
from robot.vision.egg_detector import Candidate, detect_eggs, inspect_candidates
from robot.vision.egg_size import EggDetection, classify_size
from robot.vision.frames import rgb_to_bgr

OK_BGR = (90, 200, 90)
OUT_BGR = (40, 120, 220)
TARGET_BGR = (190, 226, 240)
LINE_PX = 2


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Detect eggs in one image with the thresholds in config.py.")
    parser.add_argument("image", type=Path, help="PNG or JPEG image (BGR as saved by the capture key)")
    parser.add_argument("--rgb", action="store_true", help="the image is RGB-ordered (old signboard screenshot)")
    parser.add_argument("--debug", action="store_true",
                        help="list every candidate component with its measurements and rejection reason")
    return parser.parse_args(argv)


def output_path(image: Path) -> Path:
    """`<dir>/<stem>-detected.png` next to the input."""
    return image.with_name(f"{image.stem}-detected.png")


def describe(index: int, det: EggDetection) -> str:
    ellipse = ""
    if det.ellipse is not None:
        center_x, center_y, axis_a, axis_b, angle = det.ellipse
        ellipse = f"  ellipse=({center_x:.3f},{center_y:.3f} axes {axis_a:.3f}x{axis_b:.3f} @ {angle:.0f} deg)"
    return (f"{index}: {det.color} egg  cx={det.cx:.3f} cy={det.cy:.3f} w={det.w:.3f} h={det.h:.3f}  "
            f"spots={det.spots} area={det.area_px}px solidity={det.solidity:.3f} "
            f"border={'yes' if det.touches_border else 'no'}  size={classify_size(det)}{ellipse}")


def describe_candidate(candidate: Candidate) -> str:
    """One --debug line per cluster: core spots, d_med, window; then bbox, area, aspect, scale,
    solidity, fill, border, spot pixels per color (unmeasured values as -), and the verdict."""
    measured = bool(candidate.spots)  # shape and spots are measured only past the cheap rules
    solidity = f"{candidate.solidity:.2f}" if measured else "-"
    fill = "-" if candidate.fill is None else f"{candidate.fill:.2f}"
    spots = " ".join(f"{color}={pixels}px/{count}" for color, count, pixels in candidate.spots) or "-"
    verdict = "EGG" if candidate.rejected is None else f"rejected: {candidate.rejected}"
    return (f"  cluster: {candidate.cluster_spots} spot(s) d_med={candidate.d_med:.1f} window={candidate.window}  "
            f"bbox=({candidate.x},{candidate.y},{candidate.w},{candidate.h}) area={candidate.area_px} "
            f"aspect={candidate.aspect:.2f} scale={candidate.scale:.2f} solidity={solidity} fill={fill} "
            f"border={'yes' if candidate.touches_border else 'no'} spots[{spots}]  {verdict}")


def _box(frame: Any, cx: float, cy: float, w: float, h: float) -> tuple[tuple[int, int], tuple[int, int]]:
    height, width = frame.shape[:2]
    return ((round((cx - w / 2) * width), round((cy - h / 2) * height)),
            (round((cx + w / 2) * width), round((cy + h / 2) * height)))


def annotate(frame: Any, detections: tuple[EggDetection, ...]) -> Any:
    """A copy of `frame` with the target ellipse and, per detection, a labelled box and its
    fitted ellipse."""
    out = frame.copy()
    (x0, y0), (x1, y1) = _box(out, ALIGN_TARGET_CX, ALIGN_TARGET_CY, ALIGN_TARGET_W, ALIGN_TARGET_H)
    cv2.ellipse(out, ((x0 + x1) // 2, (y0 + y1) // 2), ((x1 - x0) // 2, (y1 - y0) // 2), 0, 0, 360, TARGET_BGR, LINE_PX)
    for index, det in enumerate(detections, start=1):
        color = OK_BGR if classify_size(det) == "ok" else OUT_BGR
        top_left, bottom_right = _box(out, det.cx, det.cy, det.w, det.h)
        cv2.rectangle(out, top_left, bottom_right, color, 1)
        if det.ellipse is not None:
            center_x, center_y, axis_a, axis_b, angle = det.ellipse
            height, width = out.shape[:2]
            cv2.ellipse(out, ((center_x * width, center_y * height), (axis_a * width, axis_b * height), angle),
                        color, LINE_PX)
        cv2.putText(out, f"{index} {det.color}", (top_left[0], max(top_left[1] - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return out


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    frame = cv2.imread(str(args.image), cv2.IMREAD_COLOR) if args.image.is_file() else None
    if frame is None:
        print(f"Cannot read image: {args.image}", file=sys.stderr)
        return 2
    frame = rgb_to_bgr(frame) if args.rgb else frame
    detections = detect_eggs(frame)
    height, width = frame.shape[:2]
    print(f"{args.image}: {width}x{height}, {len(detections)} egg(s)")
    for index, det in enumerate(detections, start=1):
        print(describe(index, det))
    if args.debug:
        candidates = inspect_candidates(frame)
        print(f"{len(candidates)} spot cluster(s):")
        for candidate in candidates:
            print(describe_candidate(candidate))
    target = output_path(args.image)
    if not cv2.imwrite(str(target), annotate(frame, detections)):
        print(f"Cannot write {target}", file=sys.stderr)
        return 1
    print(f"Wrote {target}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
