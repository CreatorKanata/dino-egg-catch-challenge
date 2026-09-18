"""tests/dino_controller: Run sanitized sketch tests and validate emitted serial frames."""
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest


class JoystickTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        folder = Path(__file__).resolve().parent
        with tempfile.TemporaryDirectory(prefix="dino-joystick-tests-") as temp:
            binary = Path(temp) / "joystick-tests"
            subprocess.run(
                shlex.split(os.environ.get("CXX", "c++"))
                + ["-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
                   "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                   "-I", str(folder / "fakes"), str(folder / "test_joystick.cpp"),
                   "-o", str(binary)], check=True,
            )
            result = subprocess.run([str(binary)], check=True, capture_output=True, text=True)
        cls.lines = result.stdout.splitlines()
        cls.events = [json.loads(line) for line in cls.lines]

    def test_framing_and_sequence(self):
        self.assertEqual([e["seq"] for e in self.events], list(range(len(self.events))))
        for line, event in zip(self.lines, self.events):
            self.assertLessEqual(len(line.encode()) + 1, 256)
            self.assertEqual(event["v"], 0)
            self.assertIs(type(event["ms"]), int)
            self.assertNotIn("button", event)

    def test_startup_and_wiring(self):
        self.assertEqual(self.events[0]["type"], "ready")
        self.assertEqual(self.events[0]["device"], "dino-controller")
        self.assertEqual(self.events[0]["pins"], {"up": 26, "down": 27, "left": 21, "right": 25})
        self.assertEqual(self.events[1]["type"], "joystick")
        self.assertTrue(self.events[1]["left"])

    def test_directions_release_and_diagonal(self):
        directions = ("up", "down", "left", "right")
        samples = [e for e in self.events if e["type"] == "joystick"]
        active = [{k for k in directions if e[k]} for e in samples[:9]]
        self.assertEqual(active, [{"left"}, set(), {"up"}, {"down"}, {"right"},
                                  {"left"}, {"up", "right"}, {"up", "down"}, set()])
        for event in samples:
            self.assertTrue(all(type(event[k]) is bool for k in directions))

    def test_state_command_and_overflow(self):
        self.assertEqual(self.events[10]["type"], "joystick")
        self.assertFalse(any(self.events[10][k] for k in ("up", "down", "left", "right")))
        self.assertEqual(self.events[-2]["type"], "error")
        self.assertEqual(self.events[-2]["code"], "event_overflow")
        self.assertEqual(self.events[-1]["type"], "joystick")
        self.assertTrue(self.events[-1]["up"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
