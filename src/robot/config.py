"""src/robot/config.py: Central runtime tunables for Manual Mode and Auto Catch alignment.

Every address, port, rate, speed level, and input-role choice used by src/robot lives here,
as required by AGENTS.md. Values come from docs/lekiwi-app-development.md and the
dino-controller docs; nothing here is a newly invented hardware limit.
"""

from dataclasses import dataclass
from typing import Final, Literal


@dataclass(frozen=True)
class SpeedLevel:
    """One base speed setting: linear speed in m/s and rotation speed in deg/s."""

    xy: float
    theta: float


# --- LeKiwi host on the Raspberry Pi (verified 2026-09-24) ---------------------------------
PI_REMOTE_IP: Final = "10.102.6.48"
ROBOT_ID: Final = "dino_kiwi"
ZMQ_CMD_PORT: Final = 5555  # LeKiwiClientConfig.port_zmq_cmd
ZMQ_OBSERVATION_PORT: Final = 5556  # LeKiwiClientConfig.port_zmq_observations

# --- dino-controller (USB serial, JSON protocol v0) ----------------------------------------
# Port used in the owner's Manual Mode session on the robot (2026-09-24; the firmware docs
# record the earlier /dev/cu.usbserial-110). Override with --controller-port when the Mac
# assigns a different device name.
CONTROLLER_SERIAL_PORT: Final = "/dev/cu.usbserial-11130"
CONTROLLER_BAUD: Final = 115200
# Longest emitted frame including LF (docs/dino-controller-protocol.md, "Frame buffer").
CONTROLLER_LINE_MAX_BYTES: Final = 255
# No valid controller line for this long -> input lost -> zero velocities. This is the
# application's primary stop; the host's 500 ms command watchdog is only the backstop.
CONTROLLER_INPUT_TIMEOUT_S: Final = 0.5
# The firmware is silent while inputs are unchanged, so the reader sends `STATE` at this
# interval. Replies keep the timeout above meaningful and also retry resynchronization.
# Must stay well below CONTROLLER_INPUT_TIMEOUT_S.
CONTROLLER_STATE_POLL_INTERVAL_S: Final = 0.2

# --- Overhead camera on the Mac --------------------------------------------------------------
# Find the index with `lerobot-find-cameras opencv`. 640x480 at 30 fps matches the Pi cameras.
TOP_CAMERA_INDEX: Final = 0
TOP_CAMERA_KEY: Final = "top"
TOP_CAMERA_WIDTH: Final = 640
TOP_CAMERA_HEIGHT: Final = 480
TOP_CAMERA_FPS: Final = 30
# "bgr" (not the LeRobot default RGB) matches the Pi front/wrist frames, which LeKiwiClient
# decodes with cv2.imdecode (BGR), so all three Rerun panels share one channel order.
TOP_CAMERA_COLOR_MODE: Final = "bgr"

# --- Pi cameras (front, wrist) -------------------------------------------------------------
# Channel order of the frames LeKiwiClient returns. Verified chain in the fork: OpenCVCamera
# converts BGR->RGB for its default color_mode RGB (camera_opencv.py, configuration_opencv.py;
# config_lekiwi.py does not override it), lekiwi_host.py cv2.imencode()s that RGB array as if it
# were BGR, and LeKiwiClient._decode_image cv2.imdecode()s it without a conversion, so the client
# arrays are RGB-ordered. The parent converts them to BGR once (robot/vision/frames.py); set
# "bgr" to disable the conversion if the fork ever changes. Owner-confirmed on the signboard.
PiCameraColorOrder = Literal["rgb", "bgr"]
PI_CAMERA_COLOR_ORDER: PiCameraColorOrder = "rgb"
FRONT_CAMERA_KEY: Final = "front"
PI_CAMERA_KEYS: Final = (FRONT_CAMERA_KEY, "wrist")

# --- Arm during Manual Mode -----------------------------------------------------------------
# The fork's host writes Goal_Position for every action and fails on an empty arm key set, so
# every action carries all six arm keys. The host enables torque and position mode on connect,
# so the pose observed at startup is held until the leader arm is engaged (arm_follow.py); the
# last commanded pose is held whenever the leader is absent, faulty, or ignored (FSC, Stop).
# Arm keys are never derived from controller input.
ArmMode = Literal["hold_initial_pose"]
ARM_MODE: ArmMode = "hold_initial_pose"
ARM_KEYS: Final = (
    "arm_shoulder_pan.pos",
    "arm_shoulder_lift.pos",
    "arm_elbow_flex.pos",
    "arm_wrist_flex.pos",
    "arm_wrist_roll.pos",
    "arm_gripper.pos",
)
# The leader arm (SO100Leader.get_action) reports the same joints without the `arm_` prefix.
LEADER_KEYS: Final = tuple(key.removeprefix("arm_") for key in ARM_KEYS)

# --- Leader arm on the Mac (Manual Mode puppeteering) -------------------------------------
# Verified values from docs/lekiwi-app-development.md (working session 2026-09-24). The
# calibration file dino_leader_arm.json lives in
# ~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/.
LEADER_ARM_PORT: Final = "/dev/tty.usbmodem5A7A0179021"
LEADER_ARM_ID: Final = "dino_leader_arm"
# Slow engagement (owner decision: approach slowly, then follow in real time). The numbers are
# proposals to tune on the robot, not measurements. Joint keys are in degrees; the gripper key
# is in percent, and the same numeric rate/tolerance is applied to it (percent per second).
ARM_ENGAGE_SPEED_DEG_S: Final = 30.0
ARM_ENGAGE_TOLERANCE_DEG: Final = 3.0

# --- Control loop ----------------------------------------------------------------------------
LOOP_HZ: Final = 30

# Exactly the three LeKiwiClient speed levels (slow, medium, fast); start slow.
SPEED_LEVELS: Final = (
    SpeedLevel(xy=0.1, theta=30.0),
    SpeedLevel(xy=0.2, theta=60.0),
    SpeedLevel(xy=0.3, theta=90.0),
)
INITIAL_SPEED_INDEX: Final = 0

# --- Input roles -------------------------------------------------------------------------------
# Owner decisions (2026-09-24): the joystick translates the base (forward/back/left/right) and
# the encoder rotates it (cw = right, ccw = left); no input changes the speed level any more.
# The shaft button has no role (docs/spec/operating-modes.md, section 3: Catch moved to `Hi!`,
# stop is the second KachiButton); it stays wired and reported, reserved for a later use.
LeftRightRole = Literal["rotate", "strafe"]
EncoderRole = Literal["speed", "rotate_base"]
ShaftButtonRole = Literal["catch", "none"]

ROLE_ROTATE: Final = "rotate"  # joystick left/right turns the base (theta.vel)
ROLE_STRAFE: Final = "strafe"  # joystick left/right moves sideways (y.vel)
ROLE_ROTATE_BASE: Final = "rotate_base"  # encoder cw = rotate right, ccw = rotate left
ROLE_SPEED: Final = "speed"  # encoder cw = faster level, ccw = slower level (kept available)
ROLE_CATCH: Final = "catch"  # shaft button requests Catch; the base stops while held (kept available)
ROLE_NONE: Final = "none"  # shaft button ignored

LEFT_RIGHT_ROLE: LeftRightRole = ROLE_STRAFE
ENCODER_ROLE: EncoderRole = ROLE_ROTATE_BASE
SHAFT_BUTTON_ROLE: ShaftButtonRole = ROLE_NONE

# Each click adds a rotation budget (knob angle per click x ENCODER_ROTATION_SCALE) that is
# spent at the current level's theta speed (open-loop: commanded, not measured).
# Owner decision 2026-09-24: the base turns half the knob angle per click; 1.0 would be 1:1.
ENCODER_ROTATION_SCALE: Final = 0.5
# Placeholder for a typical 20-detent module. Replace it with the counted test described in
# docs/dino-controller-encoder.md ("Detent calibration" is still pending in
# docs/dino-controller-validation.md).
ENCODER_CLICKS_PER_REVOLUTION: Final = 20
# Derived, never set by hand: 20 detents x 0.5 -> 9 degrees per click.
ENCODER_DEGREES_PER_STEP: Final = 360.0 / ENCODER_CLICKS_PER_REVOLUTION * ENCODER_ROTATION_SCALE
# One full knob turn may be queued. Runaway guard, tune on the robot.
ENCODER_MAX_PENDING_DEG: Final = 360.0

# --- Attendee signboard (pygame window; owner decision 2026-09-24) -------------------------
@dataclass(frozen=True)
class SignboardTheme:
    """RGB colors for the signboard."""

    background: tuple[int, int, int]
    frame: tuple[int, int, int]
    text: tuple[int, int, int]
    accent: tuple[int, int, int]
    warning: tuple[int, int, int]
    ok: tuple[int, int, int]


# Placeholder Jurassic palette; artwork (wood-sign frames, fonts, footprint icons) comes later
# as image assets.
DEFAULT_THEME: Final = SignboardTheme(
    background=(18, 38, 24),  # dark jungle green
    frame=(92, 64, 32),  # earthy brown
    text=(240, 226, 190),  # warm cream
    accent=(220, 120, 40),  # Catch
    warning=(200, 60, 50),  # input lost, rejected Auto Catch
    ok=(90, 200, 90),  # egg at a usable size
)
SIGNBOARD_SIZE: Final = (1280, 720)
SIGNBOARD_FULLSCREEN: Final = False
SIGNBOARD_TITLE: Final = "Dino Egg Catch Challenge"
SIGNBOARD_MARGIN: Final = 16
SIGNBOARD_STATUS_HEIGHT: Final = 96
SIGNBOARD_FRAME_WIDTH: Final = 4
SIGNBOARD_FONT_SIZE: Final = 36  # pygame default font for now
SIGNBOARD_SMALL_FONT_SIZE: Final = 24
# Camera keys in display order: overhead view large, Pi cameras stacked on the right.
SIGNBOARD_MAIN_CAMERA: Final = TOP_CAMERA_KEY
SIGNBOARD_SIDE_CAMERAS: Final = PI_CAMERA_KEYS
# The signboard runs in its own interpreter (opencv and pygame each bundle SDL2 on macOS).
SIGNBOARD_PROTOCOL_VERSION: Final = 1
SIGNBOARD_CHILD_POLL_S: Final = 0.05  # child waits this long for a packet before pumping events
SIGNBOARD_CHILD_EXIT_TIMEOUT_S: Final = 2.0  # parent waits this long for the child to exit

# --- KachiButton phrases (typed into the focused signboard window; docs/spec/operating-modes.md) --
# (typed phrase, command). Matching is exact and case-sensitive, including spaces and "!".
KACHI_PHRASES: Final = (("Go Go!", "mode_toggle"), ("Hi!", "hi"), ("Thx", "thx"), ("Stop", "stop"))
KACHI_PHRASE_GAP_S: Final = 1.0  # characters further apart than this never form one phrase
KACHI_BUFFER_MAX: Final = 32  # trailing characters kept while waiting for a phrase to complete
NOTICE_SECONDS: Final = 3.0  # how long a signboard notice ("STOP", "not available yet") stays

# --- Operator visualization (Rerun, opt-in) ------------------------------------------------
RERUN_SESSION_NAME: Final = "dino_drive_mode"

# --- Auto Catch step 1: front-camera egg detection and base alignment (Phase 2) --------------
# Placeholders, calibrate at the venue with a capture (`c` in the signboard) and
# `python -m robot.vision.inspect`. The detector values below were checked against the robot's front
# captures 20260924-220742 (best position), 20260924-220336 (egg with the pink basket behind it),
# 20260924-220404 (same egg, no basket), and 20260924-222325 (egg farther away, bluish lower half),
# and against the earlier channel-swapped screenshot; body S <= 50 keeps the white pipe at the left
# edge out of the egg (S <= 60 already merges them), the gap repair below handles the bluish white.
# HSV uses OpenCV ranges: H 0-179, S and V 0-255. Each color maps to one or more (low, high) ranges.
# Body: white incl. the shadowed lower half (V down to 40), but not the blue tarp's highlights (S).
EGG_BODY_HSV: Final = ((0, 0, 40), (179, 50, 255))
EGG_SPOT_HSV: Final = {
    "green": (((35, 60, 15), (100, 255, 255)),),  # teal-looking spots (H up to 100), shadowed down to V 15
    "blue": (((101, 120, 90), (130, 255, 255)),),  # S/V floor keeps the blue tarp (S 90-150, V < 90) out
    "red": (((0, 150, 60), (10, 255, 255)), ((170, 150, 60), (179, 255, 255))),  # S >= 150: not the pink basket
}
# Pink basket (measured on the robot: H 165-179, S 90-170, V 48-68). Excluded from the body and spot
# masks so the basket never joins an egg or adds spots. Auto Release will reuse it for its detector.
BASKET_HSV: Final = ((150, 60, 40), (179, 200, 255))
EGG_MIN_AREA_PX: Final = 1500
EGG_ASPECT_RANGE: Final = (0.5, 2.2)  # bbox width / height
EGG_MIN_SPOTS: Final = 1
EGG_OPEN_KERNEL_PX: Final = 9  # removes thin clutter (lines, tarp sparkles) before components form
EGG_CLOSE_KERNEL_PX: Final = 15
# Shape test runs on the outline closed with max(EGG_CLOSE_KERNEL_PX, fraction * min(w, h)) px, so
# bites from glare or bluish tarp reflections on the white (capture 20260924-222325) do not fail it.
# 0.14-0.24 gave one egg on every check capture; 0.12 missed the far egg, 0.26 accepted a false
# egg at the left edge of 20260924-222325.
EGG_REPAIR_KERNEL_FRACTION: Final = 0.20
EGG_MIN_SPOT_AREA_PX: Final = 40
EGG_MIN_SOLIDITY: Final = 0.85  # contour area / convex hull area: an egg outline is convex
# Contour area / fitted-ellipse area; only for components clear of the frame border (a partly
# visible egg is a truncated ellipse, still convex, so it keeps only the solidity test).
EGG_ELLIPSE_FILL_RANGE: Final = (0.75, 1.25)
EGG_BORDER_MARGIN_PX: Final = 2  # bbox closer than this to an edge = touches the border
# Best position: the egg pose in the front image that the pick policy starts from, normalized to
# the frame (cx, cy: bbox center; w, h: bbox size). The controller uses cx and h only; w only
# shapes the signboard's egg-outline guide. Values = this detector's own measurement (cx 0.485,
# cy 0.526, w 0.617, h 0.594) of capture 20260924-220742, taken by the owner with the egg at the
# best position on 2026-09-24, so target and detector agree. The box excludes the egg's shadowed
# underside. Re-capture (`c`) and update these if the camera or the pick start pose changes.
ALIGN_TARGET_CX: Final = 0.49
ALIGN_TARGET_CY: Final = 0.53
ALIGN_TARGET_W: Final = 0.62
ALIGN_TARGET_H: Final = 0.59
ALIGN_TOL_CX: Final = 0.05
ALIGN_TOL_H: Final = 0.08
# Precondition at the Hi! press: normalized bbox height window.
AUTO_CATCH_MIN_EGG_H: Final = 0.15  # below: "Egg too far"
AUTO_CATCH_MAX_EGG_H: Final = 0.85  # above: "Egg too close"
# Tapered controller (owner request 2026-09-24: slow down progressively near the egg; the first
# version drove too fast and oscillated). Per axis: ALIGN_MAX_XY * clamp(error / full-speed error,
# -1, 1), 0 inside the tolerance, and commands below ALIGN_MIN_XY become 0 so the base does not
# creep. cx and h are smoothed (EMA, ALIGN_SMOOTHING = weight of the new sample) and every
# command changes by at most ALIGN_MAX_ACCEL * dt per frame. The Pi JPEG stream adds ~100-200 ms
# of latency, which is why the speeds stay low.
ALIGN_MAX_XY: Final = 0.06  # m/s, below the slow level (SPEED_LEVELS[0].xy = 0.1)
ALIGN_FULL_SPEED_ERROR_CX: Final = 0.18  # normalized cx error at which ALIGN_MAX_XY is reached
ALIGN_FULL_SPEED_ERROR_H: Final = 0.30  # normalized height error at which ALIGN_MAX_XY is reached
ALIGN_MIN_XY: Final = 0.015  # m/s
ALIGN_SMOOTHING: Final = 0.5
ALIGN_MAX_ACCEL: Final = 0.15  # m/s^2
ALIGN_DONE_FRAMES: Final = 10  # consecutive in-tolerance frames -> aligned
ALIGN_TIMEOUT_S: Final = 15.0
ALIGN_LOST_FRAMES: Final = 30  # consecutive frames without an egg (1 s) -> "Egg lost"; steers on meanwhile
DETECT_TIMING_FRAMES: Final = 30  # detector time is averaged over this many frames and logged once
# Reference captures (`c` key in the signboard); the directory is git-ignored.
CAPTURE_DIR: Final = "captures"
CAPTURE_KEY: Final = "c"
CAPTURE_COMMAND: Final = "capture"  # signboard -> parent command line for the capture key

if type(ENCODER_CLICKS_PER_REVOLUTION) is not int or ENCODER_CLICKS_PER_REVOLUTION <= 0:
    raise ValueError("ENCODER_CLICKS_PER_REVOLUTION must be a positive int")
if not ENCODER_ROTATION_SCALE > 0:
    raise ValueError("ENCODER_ROTATION_SCALE must be positive")
if not (0 < ENCODER_DEGREES_PER_STEP <= ENCODER_MAX_PENDING_DEG):
    raise ValueError("Encoder rotation needs 0 < ENCODER_DEGREES_PER_STEP <= ENCODER_MAX_PENDING_DEG")
if CONTROLLER_STATE_POLL_INTERVAL_S >= CONTROLLER_INPUT_TIMEOUT_S:
    raise ValueError("CONTROLLER_STATE_POLL_INTERVAL_S must be below CONTROLLER_INPUT_TIMEOUT_S")
if not (ARM_ENGAGE_SPEED_DEG_S > 0 and ARM_ENGAGE_TOLERANCE_DEG > 0):
    raise ValueError("ARM_ENGAGE_SPEED_DEG_S and ARM_ENGAGE_TOLERANCE_DEG must be positive")
if KACHI_BUFFER_MAX < max(len(phrase) for phrase, _ in KACHI_PHRASES):
    raise ValueError("KACHI_BUFFER_MAX must hold the longest KachiButton phrase")
if not (ALIGN_TOL_CX > 0 and ALIGN_TOL_H > 0):
    raise ValueError("ALIGN_TOL_CX and ALIGN_TOL_H must be positive")
if not (0 < AUTO_CATCH_MIN_EGG_H < ALIGN_TARGET_H < AUTO_CATCH_MAX_EGG_H <= 1):
    raise ValueError("Need 0 < AUTO_CATCH_MIN_EGG_H < ALIGN_TARGET_H < AUTO_CATCH_MAX_EGG_H <= 1")
if not (0 <= ALIGN_MIN_XY < ALIGN_MAX_XY and ALIGN_MAX_ACCEL > 0 and 0 < ALIGN_SMOOTHING <= 1):
    raise ValueError("Need 0 <= ALIGN_MIN_XY < ALIGN_MAX_XY, ALIGN_MAX_ACCEL > 0, 0 < ALIGN_SMOOTHING <= 1")
if not (ALIGN_TOL_CX < ALIGN_FULL_SPEED_ERROR_CX and ALIGN_TOL_H < ALIGN_FULL_SPEED_ERROR_H):
    raise ValueError("Full-speed errors must exceed the tolerances")
# Stall guard: just outside a tolerance the tapered command (ALIGN_MAX_XY * tol / full-speed error)
# must still reach ALIGN_MIN_XY; otherwise the deadband zeroes it while the egg is not "done", and
# the base sits still until ALIGN_TIMEOUT_S (inside a tolerance the axis is 0 anyway).
if (ALIGN_MAX_XY * ALIGN_TOL_CX / ALIGN_FULL_SPEED_ERROR_CX < ALIGN_MIN_XY
        or ALIGN_MAX_XY * ALIGN_TOL_H / ALIGN_FULL_SPEED_ERROR_H < ALIGN_MIN_XY):
    raise ValueError("Need ALIGN_MAX_XY * tolerance / full-speed error >= ALIGN_MIN_XY on both axes")
if min(ALIGN_DONE_FRAMES, ALIGN_LOST_FRAMES, DETECT_TIMING_FRAMES) < 1 or not ALIGN_TIMEOUT_S > 0:
    raise ValueError("Alignment frame counts must be >= 1 and ALIGN_TIMEOUT_S positive")
if len(CAPTURE_KEY) != 1 or any(CAPTURE_KEY.lower() in phrase.lower() for phrase, _ in KACHI_PHRASES):
    raise ValueError("CAPTURE_KEY must be one character that no KachiButton phrase contains")
