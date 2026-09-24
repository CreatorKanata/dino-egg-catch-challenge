"""src/robot/vision/capture.py: Save a reference capture (raw frames + detections) on the `c` key.

The owner records the real "best position" frame and venue calibration frames with this: the
current front, wrist, and top frames (BGR, without overlays) become PNGs in CAPTURE_DIR next to
a JSON file with the detections (normalized bbox, color) and the mode. Files are named
`<YYYYmmdd-HHMMSS>-<camera>.png` and `<stamp>.json`; a second capture within the same second
gets a numeric suffix instead of overwriting. OpenCV is imported lazily (see __init__).
"""

from collections.abc import Mapping, Sequence
from dataclasses import asdict
import json
from pathlib import Path
import time
from typing import Any

from robot.config import CAPTURE_DIR
from robot.vision.egg_size import EggDetection, classify_size

CAPTURE_CAMERAS = ("front", "wrist", "top")
STAMP_FORMAT = "%Y%m%d-%H%M%S"


def capture_record(stamp: str, detections: Sequence[EggDetection], mode: str, saved: Sequence[str]) -> dict[str, Any]:
    """The JSON body: stamp, mode, saved cameras, and each detection with its size class."""
    return {
        "stamp": stamp,
        "mode": mode,
        "frames": list(saved),
        "detections": [{**asdict(det), "size": classify_size(det)} for det in detections],
    }


def _free_stamp(directory: Path, stamp: str) -> str:
    candidate, suffix = stamp, 1
    while (directory / f"{candidate}.json").exists():
        candidate, suffix = f"{stamp}-{suffix}", suffix + 1
    return candidate


def save_capture(
    frames: Mapping[str, Any | None],
    detections: Sequence[EggDetection],
    mode: str,
    directory: str | Path = CAPTURE_DIR,
    local_time: time.struct_time | None = None,
) -> Path:
    """Write the PNGs and the JSON; return the JSON path. Raises OSError when a write fails."""
    import cv2  # lazy: see the module docstring

    folder = Path(directory)
    folder.mkdir(parents=True, exist_ok=True)
    stamp = _free_stamp(folder, time.strftime(STAMP_FORMAT, local_time or time.localtime()))
    saved = tuple(name for name in CAPTURE_CAMERAS if frames.get(name) is not None)
    for name in saved:
        path = folder / f"{stamp}-{name}.png"
        if not cv2.imwrite(str(path), frames[name]):
            raise OSError(f"Could not write {path}")
    record_path = folder / f"{stamp}.json"
    record_path.write_text(json.dumps(capture_record(stamp, detections, mode, saved), indent=2) + "\n")
    return record_path
