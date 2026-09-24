"""src/robot/vision/frames.py: Bring the Pi camera frames into BGR order right after observe().

LeKiwiClient returns the front and wrist frames RGB-ordered (see PI_CAMERA_COLOR_ORDER in
config.py for the verified chain), while the overhead camera, the egg detector's HSV thresholds,
the signboard ("BGR" frombuffer), cv2.imwrite captures, and Rerun logging all use BGR. This is
the single place that converts. The channel swap is a numpy view reversal (the same result as
cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)), so the control loop's unit tests never load OpenCV.
"""

from collections.abc import Mapping
from typing import Any

import numpy as np

from robot.config import PI_CAMERA_COLOR_ORDER, PI_CAMERA_KEYS


def rgb_to_bgr(frame: Any) -> Any:
    """A new contiguous HxWx3 array with the channel order reversed."""
    array = np.asarray(frame)
    if array.ndim != 3 or array.shape[2] != 3:
        raise ValueError(f"Expected an HxWx3 frame, got shape {array.shape}")
    return np.ascontiguousarray(array[..., ::-1])


def normalize_observation_frames(
    observation: Mapping[str, Any],
    color_order: str = PI_CAMERA_COLOR_ORDER,
    keys: tuple[str, ...] = PI_CAMERA_KEYS,
) -> dict[str, Any]:
    """A new observation dict whose Pi camera frames are BGR; other entries are passed through.

    Missing or None frames stay as they are. The input mapping and its arrays are not modified.
    """
    if color_order == "bgr":
        return dict(observation)
    if color_order != "rgb":
        raise ValueError(f"Unknown Pi camera color order: {color_order!r}")
    return {
        key: rgb_to_bgr(value) if key in keys and value is not None else value
        for key, value in observation.items()
    }
