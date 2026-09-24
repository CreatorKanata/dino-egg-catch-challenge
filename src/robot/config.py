"""src/robot/config.py: Central runtime tunables for Drive Mode teleoperation.

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
# Last verified controller port per docs/dino-controller-encoder.md; override with
# --controller-port when the Mac assigns a different device name.
CONTROLLER_SERIAL_PORT: Final = "/dev/cu.usbserial-110"
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

# --- Arm during Drive Mode ------------------------------------------------------------------
# The fork's host writes Goal_Position for every action and fails on an empty arm key set, so
# every action carries all six arm keys. The host enables torque and position mode on connect,
# so holding the pose observed at startup keeps the arm where it already is. This replaces the
# previous "send nothing" default; arm keys are never derived from controller input.
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
# Still defaults pending an owner decision: the shaft-button role (Catch) and ARM_MODE above.
LeftRightRole = Literal["rotate", "strafe"]
EncoderRole = Literal["speed", "rotate_base"]
ShaftButtonRole = Literal["catch"]

ROLE_ROTATE: Final = "rotate"  # joystick left/right turns the base (theta.vel)
ROLE_STRAFE: Final = "strafe"  # joystick left/right moves sideways (y.vel)
ROLE_ROTATE_BASE: Final = "rotate_base"  # encoder cw = rotate right, ccw = rotate left
ROLE_SPEED: Final = "speed"  # encoder cw = faster level, ccw = slower level (kept available)
ROLE_CATCH: Final = "catch"  # shaft button requests Catch; the base stops, no arm motion yet

LEFT_RIGHT_ROLE: LeftRightRole = ROLE_STRAFE
ENCODER_ROLE: EncoderRole = ROLE_ROTATE_BASE
SHAFT_BUTTON_ROLE: ShaftButtonRole = ROLE_CATCH

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


# Placeholder Jurassic palette; artwork (wood-sign frames, fonts, footprint icons) comes later
# as image assets.
DEFAULT_THEME: Final = SignboardTheme(
    background=(18, 38, 24),  # dark jungle green
    frame=(92, 64, 32),  # earthy brown
    text=(240, 226, 190),  # warm cream
    accent=(220, 120, 40),  # Catch
    warning=(200, 60, 50),  # input lost
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
SIGNBOARD_SIDE_CAMERAS: Final = ("front", "wrist")
# The signboard runs in its own interpreter (opencv and pygame each bundle SDL2 on macOS).
SIGNBOARD_PROTOCOL_VERSION: Final = 1
SIGNBOARD_CHILD_POLL_S: Final = 0.05  # child waits this long for a packet before pumping events
SIGNBOARD_CHILD_EXIT_TIMEOUT_S: Final = 2.0  # parent waits this long for the child to exit

# --- Operator visualization (Rerun, opt-in) ------------------------------------------------
RERUN_SESSION_NAME: Final = "dino_drive_mode"

if type(ENCODER_CLICKS_PER_REVOLUTION) is not int or ENCODER_CLICKS_PER_REVOLUTION <= 0:
    raise ValueError("ENCODER_CLICKS_PER_REVOLUTION must be a positive int")
if not ENCODER_ROTATION_SCALE > 0:
    raise ValueError("ENCODER_ROTATION_SCALE must be positive")
if not (0 < ENCODER_DEGREES_PER_STEP <= ENCODER_MAX_PENDING_DEG):
    raise ValueError("Encoder rotation needs 0 < ENCODER_DEGREES_PER_STEP <= ENCODER_MAX_PENDING_DEG")
if CONTROLLER_STATE_POLL_INTERVAL_S >= CONTROLLER_INPUT_TIMEOUT_S:
    raise ValueError("CONTROLLER_STATE_POLL_INTERVAL_S must be below CONTROLLER_INPUT_TIMEOUT_S")
