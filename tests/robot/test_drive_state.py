"""tests/robot/test_drive_state.py: Hardware-free checks of Drive Mode state and action selection.

Covers the owner-decided roles (joystick translation, encoder rotation budget consumed at the
level's theta speed), the fixed speed level, and the Catch and input-lost stops that also
clear any pending rotation.
"""

from dataclasses import replace
import math
import unittest

from robot.config import (
    ENCODER_CLICKS_PER_REVOLUTION,
    INITIAL_SPEED_INDEX,
    ROLE_ROTATE,
    ENCODER_DEGREES_PER_STEP,
    ENCODER_MAX_PENDING_DEG,
    ENCODER_ROLE,
    ENCODER_ROTATION_SCALE,
    LEFT_RIGHT_ROLE,
    LOOP_HZ,
    ROLE_ROTATE_BASE,
    ROLE_SPEED,
    SPEED_LEVELS,
)
from robot.controller_to_action import stop_action
from robot.dino_controller_reader import INITIAL_STATE
from robot.drive_state import DriveState, consume_rotation, select_action, update_drive_state

SYNCED = replace(INITIAL_STATE, synchronized=True)
FORWARD = replace(SYNCED, up=True)
CW, CCW = 1, -1
FRAME = 1 / 30
STEP = ENCODER_DEGREES_PER_STEP
THETA = SPEED_LEVELS[INITIAL_SPEED_INDEX].theta
DEG_PER_FRAME = THETA * FRAME
FRAMES_PER_CLICK = round(STEP / DEG_PER_FRAME)  # 9 deg at 30 deg/s, 1/30 s frames -> 9


def update(drive=DriveState(), controller=SYNCED, delta=0, stale=False, dt=0.0, **options):
    return update_drive_state(drive, controller, delta, stale, dt, **options)


def act(drive, controller):
    return select_action(drive, controller, SPEED_LEVELS, LEFT_RIGHT_ROLE)


class DriveStateTests(unittest.TestCase):
    def test_default_starts_slow_stopped_and_not_rotating(self):
        drive = DriveState()
        self.assertEqual((drive.speed_index, drive.pending_rotation_deg), (0, 0.0))
        self.assertTrue(drive.input_lost)
        self.assertFalse(drive.catch_requested)
        self.assertEqual(ENCODER_ROLE, ROLE_ROTATE_BASE)
        self.assertNotEqual(ROLE_ROTATE_BASE, ROLE_ROTATE)  # encoder and joystick roles are distinct

    def test_forward_uses_level_speed(self):
        drive = update(DriveState(speed_index=2), FORWARD)
        self.assertFalse(drive.input_lost)
        self.assertEqual(act(drive, FORWARD), {"x.vel": 0.3, "y.vel": 0.0, "theta.vel": 0.0})

    def test_update_does_not_mutate(self):
        drive = DriveState()
        update(drive, delta=CW)
        self.assertEqual(drive, DriveState())

    def test_catch_request_stops_base(self):
        controller = replace(FORWARD, button=True)
        drive = update(controller=controller)
        self.assertTrue(drive.catch_requested)
        self.assertEqual(act(drive, controller), stop_action())
        released = update(drive, FORWARD)
        self.assertFalse(released.catch_requested)
        self.assertAlmostEqual(act(released, FORWARD)["x.vel"], 0.1)

    def test_stale_and_unsynchronized_input_stop_base(self):
        for controller, stale in ((FORWARD, True), (replace(FORWARD, synchronized=False), False)):
            with self.subTest(stale=stale):
                drive = update(controller=controller, stale=stale)
                self.assertTrue(drive.input_lost)
                self.assertEqual(act(drive, controller), stop_action())


class EncoderRotationTests(unittest.TestCase):
    def test_one_click_is_scaled_knob_angle(self):
        self.assertEqual(ENCODER_DEGREES_PER_STEP, 360 / ENCODER_CLICKS_PER_REVOLUTION * ENCODER_ROTATION_SCALE)
        self.assertEqual(ENCODER_ROTATION_SCALE, 0.5)
        self.assertEqual(ENCODER_MAX_PENDING_DEG, 360.0)

    def test_steps_add_signed_budget(self):
        self.assertEqual(update(delta=CW).pending_rotation_deg, -STEP)
        self.assertEqual(update(delta=CCW).pending_rotation_deg, STEP)
        self.assertEqual(update(update(delta=CW), delta=CW).pending_rotation_deg, -2 * STEP)
        self.assertEqual(update(update(delta=CCW), delta=CCW).pending_rotation_deg, 2 * STEP)

    def test_full_knob_turn_queues_scaled_base_turn(self):
        drive = DriveState()
        for _ in range(ENCODER_CLICKS_PER_REVOLUTION):
            drive = update(drive, delta=CCW)
        self.assertAlmostEqual(drive.pending_rotation_deg, 360.0 * ENCODER_ROTATION_SCALE)

    def test_budget_is_clamped(self):
        clicks = math.ceil(ENCODER_MAX_PENDING_DEG / STEP) + 5
        self.assertEqual(update(delta=clicks * CW).pending_rotation_deg, -ENCODER_MAX_PENDING_DEG)
        self.assertEqual(update(delta=clicks * CCW).pending_rotation_deg, ENCODER_MAX_PENDING_DEG)
        self.assertEqual(update(delta=CCW, max_pending_deg=STEP / 2).pending_rotation_deg, STEP / 2)

    def test_opposite_steps_cancel(self):
        self.assertEqual(update(update(delta=CW), delta=CCW).pending_rotation_deg, 0.0)

    def test_speed_index_never_changes_on_encoder_input(self):
        drive = DriveState(speed_index=1)
        for delta in (CW, CW, CCW, 5 * CW):
            drive = update(drive, delta=delta)
            self.assertEqual(drive.speed_index, 1)

    def test_speed_role_still_available(self):
        drive = update(delta=CW, encoder_role=ROLE_SPEED)
        self.assertEqual((drive.speed_index, drive.pending_rotation_deg), (1, 0.0))
        self.assertEqual(update(drive, delta=-4, encoder_role=ROLE_SPEED).speed_index, 0)

    def test_budget_is_consumed_at_theta_speed_without_overshoot(self):
        drive = update(delta=CCW)  # one click at the level's theta speed
        history = []
        for _ in range(2 * FRAMES_PER_CLICK):
            drive = update(drive, dt=FRAME)
            history.append(drive.pending_rotation_deg)
        self.assertAlmostEqual(history[FRAMES_PER_CLICK - 2], DEG_PER_FRAME)
        self.assertAlmostEqual(history[FRAMES_PER_CLICK - 1], 0.0)
        self.assertEqual(history[-1], 0.0)
        self.assertTrue(all(value >= 0.0 for value in history))

    def test_cw_click_is_consumed_in_frames_per_click(self):
        drive = update(delta=CW)
        frames = 0
        while drive.pending_rotation_deg:
            drive = update(drive, dt=FRAME)
            frames += 1
        self.assertEqual(frames, FRAMES_PER_CLICK)

    def test_last_frame_consumes_only_the_remainder(self):
        remainder = DEG_PER_FRAME / 2
        self.assertAlmostEqual(consume_rotation(DEG_PER_FRAME + remainder, THETA, FRAME), remainder)
        self.assertEqual(consume_rotation(remainder, THETA, FRAME), 0.0)
        self.assertEqual(consume_rotation(-remainder, THETA, FRAME), 0.0)
        self.assertEqual(consume_rotation(0.0, THETA, FRAME), 0.0)

    def test_dt_is_clamped(self):
        self.assertAlmostEqual(consume_rotation(STEP, THETA, 10.0), STEP - THETA * 2 / LOOP_HZ)
        self.assertEqual(consume_rotation(STEP, THETA, -1.0), STEP)

    def test_action_rotates_while_pending_and_translates_together(self):
        controller = replace(SYNCED, up=True, left=True)
        right = update(controller=controller, delta=CW)
        self.assertEqual(act(right, controller), {"x.vel": 0.1, "y.vel": 0.1, "theta.vel": -THETA})
        left = update(controller=controller, delta=CCW)
        self.assertEqual(act(left, controller)["theta.vel"], THETA)
        done = update(left, controller, dt=1.0)  # clamped dt, several frames until empty
        while done.pending_rotation_deg:
            done = update(done, controller, dt=FRAME)
        self.assertEqual(act(done, controller)["theta.vel"], 0.0)

    def test_input_lost_clears_pending_and_zeros(self):
        drive = update(controller=FORWARD, delta=CW, stale=True)
        self.assertEqual(drive.pending_rotation_deg, 0.0)
        self.assertEqual(act(drive, FORWARD), stop_action())
        self.assertEqual(update(drive, FORWARD).pending_rotation_deg, 0.0)

    def test_catch_clears_pending_and_zeros(self):
        rotating = update(controller=FORWARD, delta=CCW)
        pressed = replace(FORWARD, button=True)
        drive = update(rotating, pressed)
        self.assertEqual(drive.pending_rotation_deg, 0.0)
        self.assertEqual(act(drive, pressed), stop_action())
        self.assertEqual(act(update(drive, FORWARD), FORWARD)["theta.vel"], 0.0)


if __name__ == "__main__":
    unittest.main()
