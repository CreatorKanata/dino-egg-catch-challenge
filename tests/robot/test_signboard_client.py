"""tests/robot/test_signboard_client.py: Checks of the out-of-process signboard and its isolation.

Runs fake and real signboard children through SignboardClient (packets in, KachiButton
command lines out), and verifies in fresh interpreters that the parent never loads pygame and
the child never loads lerobot or cv2, the SDL2 separation this design exists for. The real
child uses SDL's dummy video driver.
"""

from dataclasses import replace
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import time
import unittest

import numpy as np

from robot.dino_controller_reader import INITIAL_STATE
from robot.display_status import DisplayStatus
from robot.drive_state import DriveState
from robot.signboard_client import SignboardClient

SRC = str(Path(__file__).resolve().parents[2] / "src")
DRIVE = DriveState(speed_index=1, catch_requested=False, input_lost=False)
CONTROLLER = replace(INITIAL_STATE, synchronized=True, up=True)
FAKE_CHILD = (
    "import sys\n"
    "from robot.signboard_protocol import CloseRequest, read_packet\n"
    "while True:\n"
    "    packet = read_packet(sys.stdin.buffer)\n"
    "    if packet is None or isinstance(packet, CloseRequest):\n"
    "        sys.exit(0)\n"
)
COMMAND_CHILD = (
    "import sys\n"
    "from robot.signboard_protocol import CloseRequest, encode_command, read_packet\n"
    "sys.stdout.buffer.write(b'not json\\n' + encode_command('stop') + encode_command('hi'))\n"
    "sys.stdout.buffer.flush()\n"
    "while True:\n"
    "    packet = read_packet(sys.stdin.buffer)\n"
    "    if packet is None or isinstance(packet, CloseRequest):\n"
    "        sys.exit(0)\n"
)
HEADLESS_ENV = {**os.environ, "SDL_VIDEODRIVER": "dummy"}
STATUS = DisplayStatus(mode="fsc", notice="FSC", arm_status="holding")


def frames(fill=0):
    frame = np.full((48, 64, 3), fill, dtype=np.uint8)
    return {"top": frame, "front": None, "wrist": frame[:, ::-1]}


def wait_until(condition, timeout=10.0):
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        time.sleep(0.02)
    return condition()


def run_isolated(code):
    env = {**HEADLESS_ENV, "PYTHONPATH": SRC}
    return subprocess.run([sys.executable, "-c", code], env=env, capture_output=True, timeout=120)


class SignboardClientTests(unittest.TestCase):
    def test_fake_child_receives_packets_and_exits_cleanly(self):
        client = SignboardClient(command=[sys.executable, "-c", FAKE_CHILD])
        client.open()
        self.addCleanup(client.close)
        client.render(frames(1), DRIVE, CONTROLLER)
        client.render(frames(2), DRIVE, CONTROLLER)
        self.assertTrue(client.pump())
        client.close()
        self.assertEqual(client.returncode, 0)
        self.assertFalse(client.pump())
        client.close()  # idempotent

    def test_commands_from_child_are_polled_and_malformed_lines_dropped(self):
        client = SignboardClient(command=[sys.executable, "-c", COMMAND_CHILD])
        client.open()
        self.addCleanup(client.close)
        client.render(frames(1), DRIVE, CONTROLLER, STATUS)
        received = []
        self.assertTrue(wait_until(lambda: received.extend(client.poll_commands()) or len(received) >= 2))
        self.assertEqual(received, ["stop", "hi"])
        self.assertEqual(client.poll_commands(), ())
        client.close()
        self.assertEqual(client.returncode, 0)
        self.assertFalse(client._reader.is_alive())

    def test_poll_commands_before_open_is_empty(self):
        self.assertEqual(SignboardClient(command=[sys.executable, "-c", FAKE_CHILD]).poll_commands(), ())

    def test_dead_child_is_reported_without_raising(self):
        client = SignboardClient(command=[sys.executable, "-c", "import sys; sys.exit(0)"])
        client.open()
        self.addCleanup(client.close)
        self.assertTrue(wait_until(lambda: not client.pump()))
        client.render(frames(), DRIVE, CONTROLLER)
        client.render(frames(), DRIVE, CONTROLLER)
        self.assertFalse(client.pump())
        client.close()
        self.assertEqual(client.returncode, 0)

    def test_render_and_pump_before_open(self):
        client = SignboardClient(command=[sys.executable, "-c", FAKE_CHILD])
        client.render(frames(), DRIVE, CONTROLLER)
        self.assertFalse(client.pump())
        client.close()


class IsolationTests(unittest.TestCase):
    def test_child_module_never_loads_lerobot_or_cv2(self):
        result = run_isolated(
            "import robot.signboard_process, sys\n"
            "assert 'cv2' not in sys.modules and 'lerobot' not in sys.modules, 'child imports lerobot/cv2'\n"
        )
        self.assertEqual(result.returncode, 0, result.stderr.decode())
        self.assertNotIn(b"objc[", result.stderr)

    def test_parent_module_never_loads_pygame(self):
        result = run_isolated(
            "import robot.teleop_drive, sys\nassert 'pygame' not in sys.modules, 'parent imports pygame'\n"
        )
        self.assertEqual(result.returncode, 0, result.stderr.decode())
        self.assertNotIn(b"objc[", result.stderr)

    def test_pure_decision_modules_are_stdlib_only(self):
        result = run_isolated(
            "import sys\n"
            "import robot.align, robot.mode_manager, robot.manual_mode, robot.display_status\n"
            "import robot.vision.egg_size, robot.vision.timing, robot.vision.align_trace\n"
            "import robot.auto_release, robot.arm_motions, robot.arm_store, robot.arm_follow\n"
            "import robot.vision.basket_size, robot.vision.config_vision, robot.auto_catch\n"
            "loaded = [name for name in ('cv2', 'numpy', 'pygame', 'lerobot') if name in sys.modules]\n"
            "assert not loaded, loaded\n"
        )
        self.assertEqual(result.returncode, 0, result.stderr.decode())

    def test_loop_and_vision_imports_do_not_load_cv2(self):
        result = run_isolated(
            "import sys\n"
            "import robot.drive_loop, robot.vision.egg_detector, robot.vision.capture, robot.vision.frames\n"
            "import robot.vision.top_frame, robot.vision.egg_masks, robot.vision.spot_edges\n"
            "import robot.vision.basket_detector, robot.vision.wrist_check\n"
            "assert 'cv2' not in sys.modules and 'pygame' not in sys.modules\n"
        )
        self.assertEqual(result.returncode, 0, result.stderr.decode())


class HeadlessEndToEndTests(unittest.TestCase):
    def test_real_child_renders_and_exits_zero(self):
        if importlib.util.find_spec("pygame") is None:
            self.skipTest("pygame is not installed")
        client = SignboardClient(size=(480, 320), env=HEADLESS_ENV, exit_timeout_s=30.0)
        client.open()
        self.addCleanup(client.close)
        for fill in (10, 120, 250):
            client.render(frames(fill), DRIVE, CONTROLLER, STATUS)
            time.sleep(0.05)
        self.assertTrue(client.pump())
        self.assertEqual(client.poll_commands(), ())  # nothing typed, nothing on stdout
        client.close()
        self.assertEqual(client.returncode, 0)


if __name__ == "__main__":
    unittest.main()
