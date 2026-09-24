"""src/robot/controller_to_action.py: Map cached controller levels to LeKiwi base velocities.

Pure, stdlib-only functions used by Manual Mode (base driving). Sign conventions follow
LeKiwiClient._from_keyboard_to_base_action (forward = +x, left = +y, rotate left = +theta),
so the joystick behaves like the reference keyboard teleoperation.
"""

from robot.config import ROLE_ROTATE, ROLE_STRAFE, SpeedLevel
from robot.dino_controller_reader import ControllerState

BASE_KEYS = ("x.vel", "y.vel", "theta.vel")


def stop_action() -> dict[str, float]:
    """Zero velocity for every base axis."""
    return {key: 0.0 for key in BASE_KEYS}


def _axis(positive: bool, negative: bool) -> float:
    """+1, -1, or 0; contradictory inputs cancel."""
    return float(positive) - float(negative)


def base_action(
    state: ControllerState,
    speed: SpeedLevel,
    left_right_role: str,
    theta_vel: float | None = None,
) -> dict[str, float]:
    """Return {x.vel, y.vel, theta.vel} for the held directions; zeros until synchronized.

    `theta_vel`, when given (encoder rotation), overrides the rotation from the joystick role.
    """
    if not state.synchronized:
        return stop_action()
    forward = _axis(state.up, state.down) * speed.xy
    lateral = _axis(state.left, state.right)
    if left_right_role == ROLE_ROTATE:
        action = {"x.vel": forward, "y.vel": 0.0, "theta.vel": lateral * speed.theta}
    elif left_right_role == ROLE_STRAFE:
        action = {"x.vel": forward, "y.vel": lateral * speed.xy, "theta.vel": 0.0}
    else:
        raise ValueError(f"Unknown left/right role: {left_right_role!r}")
    return action if theta_vel is None else {**action, "theta.vel": theta_vel}


def step_speed_index(index: int, delta: int, level_count: int) -> int:
    """Move `index` by `delta`, clamped to [0, level_count - 1]."""
    return max(0, min(level_count - 1, index + delta))
