"""tests/robot/test_controller_to_action.py: Hardware-free checks of the base-velocity mapper.

Confirms the joystick -> {x.vel, y.vel, theta.vel} mapping matches the LeKiwiClient keyboard
sign convention and that unsynchronized input never produces motion.
"""

from dataclasses import replace
import unittest

from robot.config import (
    INITIAL_SPEED_INDEX,
    LEFT_RIGHT_ROLE,
    ROLE_ROTATE,
    ROLE_STRAFE,
    SPEED_LEVELS,
    SpeedLevel,
)
from robot.controller_to_action import base_action, step_speed_index, stop_action
from robot.dino_controller_reader import INITIAL_STATE

SYNCED = replace(INITIAL_STATE, synchronized=True)
SLOW, MEDIUM, FAST = SPEED_LEVELS


def held(**directions):
    return replace(SYNCED, **directions)


class BaseActionTests(unittest.TestCase):
    def assertAction(self, state, expected, speed=SLOW, role=ROLE_ROTATE):
        action = base_action(state, speed, role)
        self.assertEqual(set(action), {"x.vel", "y.vel", "theta.vel"})
        for key, value in zip(("x.vel", "y.vel", "theta.vel"), expected):
            self.assertAlmostEqual(action[key], value, msg=key)

    def test_idle_is_zero(self):
        self.assertAction(SYNCED, (0.0, 0.0, 0.0))

    def test_forward_and_back(self):
        self.assertAction(held(up=True), (0.1, 0.0, 0.0))
        self.assertAction(held(down=True), (-0.1, 0.0, 0.0))

    def test_rotate_role_turns(self):
        self.assertAction(held(left=True), (0.0, 0.0, 30.0))
        self.assertAction(held(right=True), (0.0, 0.0, -30.0))

    def test_diagonal_combines_axes(self):
        self.assertAction(held(up=True, right=True), (0.2, 0.0, -60.0), speed=MEDIUM)

    def test_contradictory_directions_cancel(self):
        self.assertAction(held(up=True, down=True, left=True, right=True), (0.0, 0.0, 0.0))

    def test_strafe_role_moves_sideways(self):
        self.assertAction(held(left=True), (0.0, 0.3, 0.0), speed=FAST, role=ROLE_STRAFE)
        self.assertAction(held(up=True, right=True), (0.1, -0.1, 0.0), role=ROLE_STRAFE)

    def test_unsynchronized_is_zero(self):
        state = replace(INITIAL_STATE, up=True, left=True)
        self.assertAction(state, (0.0, 0.0, 0.0), speed=FAST)
        self.assertAction(state, (0.0, 0.0, 0.0), speed=FAST, role=ROLE_STRAFE)

    def test_default_role_is_strafe_owner_decision(self):
        self.assertEqual(LEFT_RIGHT_ROLE, ROLE_STRAFE)
        self.assertAction(held(left=True), (0.0, 0.1, 0.0), role=LEFT_RIGHT_ROLE)
        self.assertAction(held(right=True), (0.0, -0.1, 0.0), role=LEFT_RIGHT_ROLE)
        self.assertAction(held(up=True, left=True), (0.1, 0.1, 0.0), role=LEFT_RIGHT_ROLE)

    def test_theta_override_combines_with_translation(self):
        action = base_action(held(up=True, right=True), SLOW, ROLE_STRAFE, theta_vel=-30.0)
        self.assertEqual(action, {"x.vel": 0.1, "y.vel": -0.1, "theta.vel": -30.0})
        rotate_role = base_action(held(left=True), SLOW, ROLE_ROTATE, theta_vel=-30.0)
        self.assertEqual(rotate_role["theta.vel"], -30.0)

    def test_theta_override_ignored_when_unsynchronized(self):
        self.assertEqual(base_action(INITIAL_STATE, SLOW, ROLE_STRAFE, theta_vel=30.0), stop_action())

    def test_unknown_role_is_rejected(self):
        with self.assertRaises(ValueError):
            base_action(held(up=True), SLOW, "spin")

    def test_stop_action_is_all_zero(self):
        self.assertEqual(stop_action(), {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0})


class SpeedLevelTests(unittest.TestCase):
    def test_levels_match_lekiwi_client(self):
        self.assertEqual(SPEED_LEVELS, (SpeedLevel(0.1, 30.0), SpeedLevel(0.2, 60.0), SpeedLevel(0.3, 90.0)))
        self.assertEqual(INITIAL_SPEED_INDEX, 0)

    def test_step_is_clamped(self):
        self.assertEqual(step_speed_index(0, 1, 3), 1)
        self.assertEqual(step_speed_index(2, 1, 3), 2)
        self.assertEqual(step_speed_index(0, -1, 3), 0)
        self.assertEqual(step_speed_index(1, 5, 3), 2)
        self.assertEqual(step_speed_index(2, -5, 3), 0)
        self.assertEqual(step_speed_index(1, 0, 3), 1)


if __name__ == "__main__":
    unittest.main()
