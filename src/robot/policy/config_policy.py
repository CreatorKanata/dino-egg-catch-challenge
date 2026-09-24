"""src/robot/policy/config_policy.py: Tunables of the Auto Catch pick policy runner.

Kept out of robot/config.py, which is at its 300-line limit (AGENTS.md: centralize runtime
tunables). The checkpoint, device, and action horizon are CLI-overridable defaults of
`python -m robot.teleop_drive`; the stop condition and the caps are proposals to tune on the robot
(no robot run yet). Stdlib-only so the pure step logic and the signboard child can import it.
"""

from typing import Final

# --- Checkpoint (--pick-policy, --policy-device) --------------------------------------------
# Hub repo id or a local checkpoint directory (the `pretrained_model` folder of a training run).
# An empty string disables the runner: Auto Catch keeps the stub ("Catch: policy not available yet").
PICK_POLICY_PATH: Final = "CreatorKanata/act_dino_pick_egg"
PICK_POLICY_DEVICE: Final = "auto"  # "auto" = mps if available, else cpu; or "cpu", "mps", "cuda"
# Observation features the runner can build from a LeKiwi observation; a checkpoint asking for
# anything else is refused at load.
PICK_POLICY_FEATURES: Final = frozenset({"observation.images.wrist", "observation.images.front", "observation.state"})

# --- Execution ----------------------------------------------------------------------------
# ACT `n_action_steps` override at load: actions executed per inference (one per loop frame), so
# the model runs once every PICK_ACTION_HORIZON frames. Clamped to the checkpoint's chunk_size.
PICK_ACTION_HORIZON: Final = 10
PICK_MAX_S: Final = 20.0  # hard time limit = the recorded episode length (--episode-time-s)
PICK_MAX_JOINT_STEP_DEG_PER_S: Final = 90.0  # per-frame change cap on every arm key, as the release playback
PICK_TIMING_LOG_INTERVAL_S: Final = 1.0  # inference time log at INFO at most this often

# --- Stop condition (proposal, tune on the robot) --------------------------------------------
# done when the proposed arm pose changed by less than PICK_SETTLE_DEG on every key for
# PICK_SETTLE_FRAMES consecutive frames, the gripper command is at or below
# PICK_GRIPPER_CLOSED_MAX, and PICK_MIN_S has elapsed. Whether the egg is really held is not checked.
PICK_SETTLE_FRAMES: Final = 15
PICK_SETTLE_DEG: Final = 1.0
PICK_MIN_S: Final = 3.0
# Percent. Placeholder: no local copy of CreatorKanata/dino_pick_egg was available to measure the
# final gripper value of the recorded episodes (they end with the egg held); measure and replace.
PICK_GRIPPER_CLOSED_MAX: Final = 5.0

if PICK_ACTION_HORIZON < 1 or PICK_SETTLE_FRAMES < 1:
    raise ValueError("PICK_ACTION_HORIZON and PICK_SETTLE_FRAMES must be at least 1")
if not 0 < PICK_MIN_S < PICK_MAX_S:
    raise ValueError("The pick stop condition needs 0 < PICK_MIN_S < PICK_MAX_S")
if min(PICK_MAX_JOINT_STEP_DEG_PER_S, PICK_SETTLE_DEG, PICK_TIMING_LOG_INTERVAL_S) <= 0:
    raise ValueError("PICK_MAX_JOINT_STEP_DEG_PER_S, PICK_SETTLE_DEG, and PICK_TIMING_LOG_INTERVAL_S must be positive")
