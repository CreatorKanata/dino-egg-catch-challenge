"""src/robot/config.py: Central runtime tunables for Manual Mode, Auto Catch, and Auto Release.

Every address, port, rate, speed level, and input-role choice used by src/robot lives here,
as required by AGENTS.md. Values come from docs/lekiwi-app-development.md and the
dino-controller docs; nothing here is a newly invented hardware limit.
"""

from dataclasses import dataclass
from pathlib import Path
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
# Find the index with `lerobot-find-cameras opencv`. The Mac camera's native mode is 1920x1080 at
# 30 fps (16:9); AVFoundation center-crops 4:3 requests such as 640x480, cutting off the tarp's
# edges (owner request 2026-09-24: show the whole field). 1280x720 is a universally supported
# 16:9 mode that keeps the full field. OpenCVCamera raises at connect if the camera does not
# deliver exactly this size; fall back to 960x540 or 640x360 then (src/robot/README.md).
TOP_CAMERA_INDEX: Final = 0
TOP_CAMERA_KEY: Final = "top"
TOP_CAMERA_WIDTH: Final = 1280
TOP_CAMERA_HEIGHT: Final = 720
TOP_CAMERA_ASPECT: Final = 16 / 9
TOP_CAMERA_FPS: Final = 30
# The top frame is downscaled once per loop frame to this width (aspect kept: 960x540) before it
# goes to the signboard, Rerun, and captures, keeping the pipe at about 1.5 MB per frame.
TOP_DISPLAY_WIDTH: Final = 960
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
WRIST_CAMERA_KEY: Final = "wrist"
PI_CAMERA_ASPECT: Final = 4 / 3  # front and wrist are 640x480 (config_lekiwi.py in the fork)
PI_CAMERA_KEYS: Final = (FRONT_CAMERA_KEY, WRIST_CAMERA_KEY)

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
    basket: tuple[int, int, int]


# Placeholder Jurassic palette; artwork (wood-sign frames, fonts, footprint icons) comes later
# as image assets.
DEFAULT_THEME: Final = SignboardTheme(
    background=(18, 38, 24),  # dark jungle green
    frame=(92, 64, 32),  # earthy brown
    text=(240, 226, 190),  # warm cream
    accent=(220, 120, 40),  # Catch
    warning=(200, 60, 50),  # input lost, rejected Auto Catch
    ok=(90, 200, 90),  # egg at a usable size
    basket=(235, 110, 185),  # pink basket outline (Auto Release)
)
SIGNBOARD_SIZE: Final = (1280, 720)
SIGNBOARD_FULLSCREEN: Final = False
SIGNBOARD_TITLE: Final = "Dino Egg Catch Challenge"
SIGNBOARD_MARGIN: Final = 16
SIGNBOARD_STATUS_HEIGHT: Final = 96  # minimum; the status bar takes the height the 16:9 main view leaves
SIGNBOARD_FRAME_WIDTH: Final = 4
SIGNBOARD_FONT_SIZE: Final = 44  # pygame default font for now
SIGNBOARD_SMALL_FONT_SIZE: Final = 28
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
# The egg detector's HSV ranges and segmentation tunables live in robot/vision/config_vision.py
# (moved to keep this file under 300 lines); the alignment and precondition values stay here.
# Best position (pick policy start pose), normalized. Distance is steered on the egg's fitted-ellipse width
# (EggDetection.ellipse_w; larger = closer): monotonic, smallest spread at the best position. Not h (shadow)
# and not the top edge (it saturates at the camera's horizon, run 2026-09-25); see src/robot/README.md.
ALIGN_TARGET_CX: Final = 0.49
ALIGN_TARGET_W_EGG: Final = 0.607  # median ellipse width of the best-position group
ALIGN_TARGET_CY: Final = 0.53  # cy, w, h only draw the guide (and h bounds the size precondition below)
ALIGN_TARGET_W: Final = 0.57  # the current detector's width reading of 220742 (2026-09-25), guide only
ALIGN_TARGET_H: Final = 0.61
ALIGN_TOL_CX: Final = 0.05
# Asymmetric window (owner 2026-09-24: closer is fine, farther is not): target - FAR <= width <= target + NEAR.
ALIGN_TOL_W_FAR: Final = 0.04  # narrower than the target by more than this -> forward
ALIGN_TOL_W_NEAR: Final = 0.10  # wider (closer) by at most this; beyond -> back up
# Precondition at the Hi! press: normalized bbox height window.
AUTO_CATCH_MIN_EGG_H: Final = 0.15  # below: "Egg too far"
AUTO_CATCH_MAX_EGG_H: Final = 0.85  # above: "Egg too close"
# Tapered controller (owner request 2026-09-24; the first version oscillated): per axis ALIGN_MAX_XY *
# clamp(error / full-speed error, -1, 1), 0 inside the tolerance, < ALIGN_MIN_XY -> 0; cx/width EMA-smoothed
# (weight of the new sample); rate-limited to ALIGN_MAX_ACCEL. The Pi stream adds ~100-200 ms latency.
ALIGN_MAX_XY: Final = 0.06  # m/s, below the slow level (SPEED_LEVELS[0].xy = 0.1)
ALIGN_FULL_SPEED_ERROR_CX: Final = 0.18  # normalized cx error at which ALIGN_MAX_XY is reached
ALIGN_FULL_SPEED_ERROR_W: Final = 0.15  # egg width error for ALIGN_MAX_XY (<= 0.15: stall guard)
ALIGN_FULL_SPEED_ERROR_H: Final = 0.08  # bbox size error for ALIGN_MAX_XY on the basket width path (config_vision)
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

# --- Arm data: release pose (home), release motion, catch pose (Phase 3), staff keys ----------
# Basket detector, basket alignment target, and tolerances: robot/vision/config_vision.py.
# Data files (JSON recorded on the robot with the keys below), relative to the repository root.
REPO_ROOT: Final = Path(__file__).resolve().parents[2]
HOME_POSE_PATH: Final = "data/arm/home_pose.json"
RELEASE_MOTION_PATH: Final = "data/arm/release_motion.json"
CATCH_POSE_PATH: Final = "data/arm/catch_pose.json"  # head down: the wrist camera sees the aligned egg
# Staff keys (KEYDOWN in the signboard, like CAPTURE_KEY). Not "h": the KachiButton types the "h"
# of "Thx" and "Hi!" as key presses too, so every Thx/Hi! would overwrite the home pose.
HOME_KEY: Final = "b"  # save the commanded arm pose as the home (base) pose
RECORD_KEY: Final = "r"  # start / stop recording the release motion
CATCH_POSE_KEY: Final = "k"  # save the commanded arm pose as the catch pose
SAVE_HOME_COMMAND: Final = "save_home"
SAVE_CATCH_COMMAND: Final = "save_catch"
TOGGLE_RECORD_COMMAND: Final = "toggle_record"
RELEASE_HOME_TIMEOUT_S: Final = 8.0  # arm approach to a recorded pose (home, catch, the motion's start)
RELEASE_PLAYBACK_SPEED: Final = 1.0  # 1.0 = recorded speed; 0.5 was used for the first robot test (2026-09-25), which the owner found slow
RELEASE_MAX_JOINT_STEP_DEG_PER_S: Final = 90.0  # playback: per-frame change cap on every joint
RECORD_MAX_S: Final = 30.0  # a release recording stops and saves itself at this length

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
if abs(TOP_CAMERA_WIDTH / TOP_CAMERA_HEIGHT - TOP_CAMERA_ASPECT) > 0.01:
    raise ValueError("TOP_CAMERA_WIDTH / TOP_CAMERA_HEIGHT must match TOP_CAMERA_ASPECT")
if not 0 < TOP_DISPLAY_WIDTH <= TOP_CAMERA_WIDTH:
    raise ValueError("Need 0 < TOP_DISPLAY_WIDTH <= TOP_CAMERA_WIDTH")
if KACHI_BUFFER_MAX < max(len(phrase) for phrase, _ in KACHI_PHRASES):
    raise ValueError("KACHI_BUFFER_MAX must hold the longest KachiButton phrase")
if not (ALIGN_TOL_CX > 0 and ALIGN_TOL_W_FAR > 0 and ALIGN_TOL_W_NEAR > 0 and 0 < ALIGN_TARGET_W_EGG < 1):
    raise ValueError("ALIGN_TOL_CX, ALIGN_TOL_W_FAR, ALIGN_TOL_W_NEAR, ALIGN_TARGET_W_EGG out of range")
if not (0 < AUTO_CATCH_MIN_EGG_H < ALIGN_TARGET_H < AUTO_CATCH_MAX_EGG_H <= 1):
    raise ValueError("Need 0 < AUTO_CATCH_MIN_EGG_H < ALIGN_TARGET_H < AUTO_CATCH_MAX_EGG_H <= 1")
if not (0 <= ALIGN_MIN_XY < ALIGN_MAX_XY and ALIGN_MAX_ACCEL > 0 and 0 < ALIGN_SMOOTHING <= 1):
    raise ValueError("Need 0 <= ALIGN_MIN_XY < ALIGN_MAX_XY, ALIGN_MAX_ACCEL > 0, 0 < ALIGN_SMOOTHING <= 1")
if not (ALIGN_TOL_CX < ALIGN_FULL_SPEED_ERROR_CX and ALIGN_TOL_W_FAR < ALIGN_FULL_SPEED_ERROR_W):
    raise ValueError("Full-speed errors must exceed ALIGN_TOL_CX and ALIGN_TOL_W_FAR")
# Stall guard: just outside every tolerance edge the tapered command (ALIGN_MAX_XY * min(1, tol /
# full-speed error)) must reach ALIGN_MIN_XY, or the deadband zeroes it while the egg is not
# "done" and the base waits for ALIGN_TIMEOUT_S. The far edge binds: 0.06 * 0.04 / 0.15 = 0.016 >= 0.015
# (0.25 would give 0.0096 and stall). Near edge 0.04 m/s, cx edge 0.0167 m/s; 1e-9 absorbs rounding.
_EDGES = ((ALIGN_TOL_CX, ALIGN_FULL_SPEED_ERROR_CX), (ALIGN_TOL_W_FAR, ALIGN_FULL_SPEED_ERROR_W),
          (ALIGN_TOL_W_NEAR, ALIGN_FULL_SPEED_ERROR_W))
if any(ALIGN_MAX_XY * min(1.0, tol / full) < ALIGN_MIN_XY - 1e-9 for tol, full in _EDGES):
    raise ValueError("Need ALIGN_MAX_XY * min(1, tolerance / full-speed error) >= ALIGN_MIN_XY at every edge")
if min(ALIGN_DONE_FRAMES, ALIGN_LOST_FRAMES, DETECT_TIMING_FRAMES) < 1 or not ALIGN_TIMEOUT_S > 0:
    raise ValueError("Alignment frame counts must be >= 1 and ALIGN_TIMEOUT_S positive")
_STAFF_KEYS = (CAPTURE_KEY, HOME_KEY, RECORD_KEY, CATCH_POSE_KEY)
if len({key.lower() for key in _STAFF_KEYS}) != len(_STAFF_KEYS) or any(
        len(key) != 1 or any(key.lower() in phrase.lower() for phrase, _ in KACHI_PHRASES) for key in _STAFF_KEYS):
    raise ValueError("Staff keys: distinct single characters that no KachiButton phrase contains")
if not 0 < RELEASE_PLAYBACK_SPEED <= 2:
    raise ValueError("RELEASE_PLAYBACK_SPEED must be in (0, 2]")
if not (RELEASE_HOME_TIMEOUT_S > 0 and RELEASE_MAX_JOINT_STEP_DEG_PER_S > 0 and RECORD_MAX_S > 0):
    raise ValueError("RELEASE_HOME_TIMEOUT_S, RELEASE_MAX_JOINT_STEP_DEG_PER_S, RECORD_MAX_S must be positive")
