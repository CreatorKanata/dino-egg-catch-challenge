"""src/robot/recording/config_recording.py: Tunables and defaults of the pick_egg demonstration recorder.

The recorder's CLI defaults, the start-pose gate limits, and the key map text live here because
robot/config.py is at its 300-line limit (AGENTS.md: centralize runtime tunables). The Pi address,
leader port and id, arm speed, and pose files still come from robot/config.py.
Values are owner decisions of 2026-09-25 (docs/lekiwi-app-development.md, section 6).
"""

from typing import Final

from robot.config import ARM_ENGAGE_SPEED_DEG_S

# --- Session defaults (CLI flags) ----------------------------------------------------------
DEFAULT_REPO_ID: Final = "CreatorKanata/dino_pick_egg"
DEFAULT_NUM_EPISODES: Final = 60
DEFAULT_EPISODE_TIME_S: Final = 20.0
DEFAULT_RESET_TIME_S: Final = 15.0
DEFAULT_FPS: Final = 30
DEFAULT_TASK: Final = "Pick up the egg with the mouth"
EGG_COLORS: Final = ("green", "red", "orange")

# --- Start-pose gate (before every episode) ------------------------------------------------
GATE_TOLERANCE_DEG: Final = 10.0  # every non-gripper leader joint within this of the catch pose
GATE_TIMEOUT_S: Final = 120.0  # leader not at the catch pose by then -> skip the episode
GATE_ANNOUNCE_INTERVAL_S: Final = 3.0  # spoken hints while waiting, at most this often
GATE_SPEED_DEG_S: Final = ARM_ENGAGE_SPEED_DEG_S  # follower approach to the catch pose

# --- Dataset and display -------------------------------------------------------------------
IMAGE_WRITER_THREADS: Final = 4  # as in the fork's examples/lekiwi/record.py
RERUN_SESSION_NAME: Final = "dino_pick_egg_record"

# --- Speech (owner trial run, 2026-09-25: the Mac's default voice was Japanese and garbled English) ---
RECORDER_VOICE: Final = "auto"  # "auto", "none" (speech off, text still logged), or a `say -v` voice name
RECORDER_VOICE_PREFERENCE: Final = ("Samantha", "Ava", "Allison", "Alex", "Karen", "Daniel", "Moira")
SPEECH_IDLE_WAIT_S: Final = 3.0  # before an episode, wait at most this long for queued speech to finish
SPEECH_CLOSE_WAIT_S: Final = 10.0  # at exit, let the last announcements play for at most this long

# Verified against the fork (utils/keyboard_input.py init_keyboard_listener and the LeKiwi client
# teleop_keys in robots/lekiwi/config_lekiwi.py) on 2026-09-25.
KEY_MAP: Final = (
    "Keys (pynput needs the macOS Accessibility permission; otherwise keep this terminal focused):",
    "  Right arrow (or n): end the episode now - once the egg is held and slightly lifted",
    "  Left arrow (or r):  discard and re-record this episode",
    "  Esc (or q):         stop the session (the dataset is saved, then uploaded unless --no-push)",
    "  Ctrl+C:             stop at once; the base is zeroed and the dataset is saved the same way",
    "Base keys (KeyboardTeleop): w/s forward/back, a/d left/right, z/x rotate, r/f speed up/down.",
    "  Keep the base parked while recording. 'r' is also the base speed-up key: prefer the left arrow.",
)
