"""src/robot/display_status.py: What the signboard shows about modes, the arm, and the egg, as data.

A frozen DisplayStatus travels from the control loop to the signboard child over the pipe
(signboard_protocol.py) and is turned into status text by signboard_layout.py. Overlays are
normalized shapes drawn on a camera view: the Auto Catch target (the egg-shaped guide on the
front view) and the best egg detection. Stdlib-only, so both processes can import it (the child
must never load LeRobot or cv2).
"""

from dataclasses import dataclass
from typing import Final, Literal

from robot.config import ALIGN_TARGET_CX, ALIGN_TARGET_CY, ALIGN_TARGET_H, ALIGN_TARGET_W, FRONT_CAMERA_KEY
from robot.mode_manager import Action, AppState, Mode, NoticeLevel
from robot.vision.egg_size import EggDetection, classify_size

ArmStatus = Literal["holding", "syncing", "following", "leader fault", "no leader"]
ARM_STATUSES: Final = ("holding", "syncing", "following", "leader fault", "no leader")
OverlayKind = Literal["egg_ok", "egg_out", "target"]
OVERLAY_KINDS: Final = ("egg_ok", "egg_out", "target")
TARGET_LABEL: Final = "place the egg here"
SIZE_LABEL: Final = {"too_small": "too far", "too_large": "too close"}


@dataclass(frozen=True)
class Overlay:
    """An ellipse on one camera view: center normalized per axis, first and second axis lengths
    divided by the frame width and height, and the first axis' rotation in degrees (OpenCV
    convention). With angle 0 it is the ellipse inscribed in a normalized bbox."""

    camera: str
    cx: float
    cy: float
    w: float
    h: float
    kind: OverlayKind
    label: str
    angle: float = 0.0


@dataclass(frozen=True)
class DisplayStatus:
    """Mode, running action, FSC voice input, notice and its level, arm status, Stop latch, overlays."""

    mode: Mode = "manual"
    action: Action = "none"
    voice_listening: bool = False
    notice: str = ""
    arm_status: ArmStatus = "holding"
    stopped: bool = False
    notice_level: NoticeLevel = "info"
    overlays: tuple[Overlay, ...] = ()


TARGET_OVERLAY: Final = Overlay(FRONT_CAMERA_KEY, ALIGN_TARGET_CX, ALIGN_TARGET_CY, ALIGN_TARGET_W, ALIGN_TARGET_H,
                                "target", TARGET_LABEL)


def egg_overlay(egg: EggDetection) -> Overlay:
    """The detection as "egg_ok" (usable size) or "egg_out", labelled with color and size class;
    its fitted ellipse when there is one, else the ellipse inscribed in its bbox."""
    size = classify_size(egg)
    kind: OverlayKind = "egg_ok" if size == "ok" else "egg_out"
    label = f"{egg.color} egg" if size == "ok" else f"{egg.color} egg: {SIZE_LABEL[size]}"
    if egg.ellipse is None:
        return Overlay(FRONT_CAMERA_KEY, egg.cx, egg.cy, egg.w, egg.h, kind, label)
    center_x, center_y, axis_a, axis_b, angle = egg.ellipse
    return Overlay(FRONT_CAMERA_KEY, center_x, center_y, axis_a, axis_b, kind, label, angle)


def front_overlays(app: AppState, egg: EggDetection | None) -> tuple[Overlay, ...]:
    """In Manual Mode: always the target guide, plus the best egg when one is detected."""
    if app.mode != "manual":
        return ()
    return (TARGET_OVERLAY,) if egg is None else (TARGET_OVERLAY, egg_overlay(egg))


def display_status(app: AppState, arm_status: ArmStatus, overlays: tuple[Overlay, ...] = ()) -> DisplayStatus:
    """The displayable part of the application state plus the arm status and overlays."""
    return DisplayStatus(
        mode=app.mode,
        action=app.action,
        voice_listening=app.voice_listening,
        notice=app.notice,
        arm_status=arm_status,
        stopped=app.stopped,
        notice_level=app.notice_level,
        overlays=overlays,
    )
