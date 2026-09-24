"""src/robot/top_camera.py: Overhead camera on the Mac for the Manual Mode view.

Thin wrapper around LeRobot's OpenCVCamera. The camera is opened by the application, not
declared in LeKiwiClientConfig.cameras (that field is for cameras streamed by the Pi).
A missing or stale frame returns None so the control loop never dies on the camera.
"""

import logging
from typing import Any

from lerobot.cameras import ColorMode
from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig

from robot.config import (
    TOP_CAMERA_COLOR_MODE,
    TOP_CAMERA_FPS,
    TOP_CAMERA_HEIGHT,
    TOP_CAMERA_INDEX,
    TOP_CAMERA_WIDTH,
)

logger = logging.getLogger(__name__)


class TopCamera:
    """Connect, fetch the latest frame (HWC, BGR like the Pi frames), and disconnect."""

    def __init__(
        self,
        index_or_path: int | str = TOP_CAMERA_INDEX,
        fps: int = TOP_CAMERA_FPS,
        width: int = TOP_CAMERA_WIDTH,
        height: int = TOP_CAMERA_HEIGHT,
    ) -> None:
        config = OpenCVCameraConfig(
            index_or_path=index_or_path,
            fps=fps,
            width=width,
            height=height,
            color_mode=ColorMode(TOP_CAMERA_COLOR_MODE),
        )
        self._camera = OpenCVCamera(config)

    @property
    def is_connected(self) -> bool:
        return self._camera.is_connected

    def connect(self) -> None:
        self._camera.connect()
        logger.info("Top camera connected: %s", self._camera)

    def read_latest(self) -> Any | None:
        """Return the newest frame, or None when no fresh frame is available."""
        try:
            return self._camera.read_latest()
        except (TimeoutError, RuntimeError) as error:
            logger.debug("Top camera frame unavailable: %s", error)
            return None

    def disconnect(self) -> None:
        if self._camera.is_connected:
            self._camera.disconnect()
            logger.info("Top camera disconnected")
