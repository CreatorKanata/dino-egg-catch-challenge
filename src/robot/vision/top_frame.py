"""src/robot/vision/top_frame.py: Downscale the 16:9 overhead frame once per loop frame.

The overhead camera is opened at 1280x720 so it keeps its full field of view (owner request,
2026-09-24; 4:3 requests are center-cropped by AVFoundation). The signboard, Rerun, and captures
get a copy downscaled to TOP_DISPLAY_WIDTH (960x540) with INTER_AREA, keeping the pipe at about
1.5 MB per frame; the raw frame stays in the loop for a later overhead tracker. OpenCV is
imported lazily and only when a resize is needed, so frames already small enough (and the loop's
unit tests) never load it.
"""

from typing import Any

from robot.config import TOP_DISPLAY_WIDTH


def downscale_to_width(frame: Any | None, width: int = TOP_DISPLAY_WIDTH) -> Any | None:
    """A copy of `frame` resized to `width` with its aspect ratio kept; frames already at most
    `width` wide (and None) are returned unchanged. The input is never modified."""
    if frame is None or frame.shape[1] <= width:
        return frame
    import cv2  # lazy: see the module docstring

    height = max(1, round(frame.shape[0] * width / frame.shape[1]))
    return cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
