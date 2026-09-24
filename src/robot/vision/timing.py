"""src/robot/vision/timing.py: Average the egg detector's time over the first frames, log once.

The spec asks for one INFO line with the detector cost at startup, so the loop can be judged
against its 30 Hz budget without per-frame log noise. Frozen state and stdlib-only.
"""

from dataclasses import dataclass
import logging

from robot.config import DETECT_TIMING_FRAMES

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class DetectTiming:
    """Frames measured so far and their total detector time in seconds."""

    frames: int = 0
    total_s: float = 0.0


def record_detect_time(timing: DetectTiming, elapsed_s: float, window: int = DETECT_TIMING_FRAMES) -> DetectTiming:
    """Add one measurement; log the average exactly once, when `window` frames are reached."""
    if timing.frames >= window:
        return timing
    updated = DetectTiming(frames=timing.frames + 1, total_s=timing.total_s + max(elapsed_s, 0.0))
    if updated.frames == window:
        logger.info("Egg detector: %.1f ms per frame (average of %d frames)",
                    1000.0 * updated.total_s / window, window)
    return updated
