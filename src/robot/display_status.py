"""src/robot/display_status.py: What the signboard shows about modes, the arm, and the egg, as data.

A frozen DisplayStatus (including the Auto Catch phase for the mode line) travels from the control loop to the signboard child over the pipe
(signboard_protocol.py) and is turned into status text by signboard_layout.py. Overlays are
normalized shapes drawn on a camera view: the Auto Catch target (the egg-shaped guide on the
front view), the best egg detection, the pink basket (a rectangle), and, only while Auto Release
runs, its basket target (a thin rectangle). Stdlib-only, so both processes can import it (the
child must never load LeRobot or cv2).
"""

from dataclasses import dataclass
from typing import Final, Literal

from robot.auto_catch import CatchPhase
from robot.auto_release import release_progress
from robot.config import ALIGN_TARGET_CX, ALIGN_TARGET_CY, ALIGN_TARGET_H, ALIGN_TARGET_W, FRONT_CAMERA_KEY
from robot.mode_manager import Action, AppState, Mode, NoticeLevel
from robot.vision.basket_size import BasketDetection
from robot.vision.config_vision import RELEASE_TARGET_CX, RELEASE_TARGET_CY, RELEASE_TARGET_H, RELEASE_TARGET_W
from robot.vision.egg_size import EggDetection, classify_size

ArmStatus = Literal["holding", "syncing", "following", "leader fault", "no leader", "auto release", "auto catch"]
ARM_STATUSES: Final = ("holding", "syncing", "following", "leader fault", "no leader", "auto release", "auto catch")
# Ellipses: the egg target and detections. Rectangles: the basket and the Auto Release target.
OverlayKind = Literal["egg_ok", "egg_out", "target", "basket", "release_target"]
OVERLAY_KINDS: Final = ("egg_ok", "egg_out", "target", "basket", "release_target")
RECT_KINDS: Final = ("basket", "release_target")
TARGET_LABEL: Final = "place the egg here"
BASKET_LABEL: Final = "basket"
SIZE_LABEL: Final = {"too_small": "too far", "too_large": "too close"}


@dataclass(frozen=True)
class Overlay:
    """An ellipse on one camera view: center normalized per axis, first and second axis lengths
    divided by the frame width and height, and the first axis' rotation in degrees (OpenCV
    convention). With angle 0 it is the ellipse inscribed in a normalized bbox. RECT_KINDS are
    drawn as that bbox instead (angle 0)."""

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
    """Mode, running action, FSC voice input, notice and its level, arm status, Stop latch, overlays,
    seconds of release motion recorded so far (None when not recording), Auto Release playback
    progress 0..1 (None when not playing), and the Auto Catch phase ("" when it is not running)."""

    mode: Mode = "manual"
    action: Action = "none"
    voice_listening: bool = False
    notice: str = ""
    arm_status: ArmStatus = "holding"
    stopped: bool = False
    notice_level: NoticeLevel = "info"
    overlays: tuple[Overlay, ...] = ()
    recording_s: float | None = None
    progress: float | None = None
    phase: CatchPhase | Literal[""] = ""


TARGET_OVERLAY: Final = Overlay(FRONT_CAMERA_KEY, ALIGN_TARGET_CX, ALIGN_TARGET_CY, ALIGN_TARGET_W, ALIGN_TARGET_H,
                                "target", TARGET_LABEL)
RELEASE_TARGET_OVERLAY: Final = Overlay(FRONT_CAMERA_KEY, RELEASE_TARGET_CX, RELEASE_TARGET_CY, RELEASE_TARGET_W,
                                        RELEASE_TARGET_H, "release_target", "")


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


def basket_overlay(basket: BasketDetection) -> Overlay:
    """The basket bbox as a labelled rectangle."""
    return Overlay(FRONT_CAMERA_KEY, basket.cx, basket.cy, basket.w, basket.h, "basket", BASKET_LABEL)


def front_overlays(
    app: AppState, egg: EggDetection | None, basket: BasketDetection | None = None
) -> tuple[Overlay, ...]:
    """In Manual Mode: always the egg target guide, the best egg and the basket when detected, and
    the basket target only while Auto Release runs (it would clutter the egg view otherwise)."""
    if app.mode != "manual":
        return ()
    return (TARGET_OVERLAY,
            *((egg_overlay(egg),) if egg is not None else ()),
            *((RELEASE_TARGET_OVERLAY,) if app.action == "auto_release" else ()),
            *((basket_overlay(basket),) if basket is not None else ()))


def display_status(
    app: AppState, arm_status: ArmStatus, overlays: tuple[Overlay, ...] = (), recording_s: float | None = None
) -> DisplayStatus:
    """The displayable part of the application state plus the arm status, overlays, the recording
    time, the Auto Release playback progress, and the Auto Catch phase."""
    return DisplayStatus(
        mode=app.mode,
        action=app.action,
        voice_listening=app.voice_listening,
        notice=app.notice,
        arm_status=arm_status,
        stopped=app.stopped,
        notice_level=app.notice_level,
        overlays=overlays,
        recording_s=recording_s,
        progress=release_progress(app.release) if app.action == "auto_release" else None,
        phase=app.catch.phase if app.action == "auto_catch" else "",
    )
