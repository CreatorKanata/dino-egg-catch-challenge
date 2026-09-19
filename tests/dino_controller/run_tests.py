"""tests/dino_controller: Run sanitized sketch tests and validate emitted serial frames."""
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest


def build_events(source):
    folder = Path(__file__).resolve().parent
    with tempfile.TemporaryDirectory(prefix="dino-controller-tests-") as temp:
        binary = Path(temp) / "controller-tests"
        subprocess.run(
            shlex.split(os.environ.get("CXX", "c++"))
            + ["-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
               "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
               "-I", str(folder / "fakes"), str(folder / source), "-o", str(binary)], check=True,
        )
        result = subprocess.run([str(binary)], check=True, capture_output=True, text=True)
    lines = result.stdout.splitlines()
    return lines, [json.loads(line) for line in lines]


class JoystickTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lines, cls.events = build_events("test_joystick.cpp")

    def test_framing_and_sequence(self):
        self.assertEqual([e["seq"] for e in self.events], list(range(len(self.events))))
        for line, event in zip(self.lines, self.events):
            self.assertLessEqual(len(line.encode()) + 1, 256)
            self.assertEqual(event["v"], 0)
            self.assertIs(type(event["ms"]), int)
            if event["type"] == "state":
                self.assertIs(type(event["button"]), bool)

    def test_startup_and_wiring(self):
        self.assertEqual(self.events[0]["type"], "ready")
        self.assertEqual(self.events[0]["device"], "dino-controller")
        self.assertEqual(self.events[0]["pins"], {"up": 26, "down": 27, "left": 21, "right": 25})
        self.assertEqual(self.events[1]["type"], "state")
        self.assertTrue(self.events[1]["left"])

    def test_directions_release_and_diagonal(self):
        directions = ("up", "down", "left", "right")
        samples = [e for e in self.events if e["type"] in ("state", "joystick")]
        active = [{k for k in directions if e[k]} for e in samples[:9]]
        self.assertEqual(active, [{"left"}, set(), {"up"}, {"down"}, {"right"},
                                  {"left"}, {"up", "right"}, {"up", "down"}, set()])
        for event in samples:
            self.assertTrue(all(type(event[k]) is bool for k in directions))

    def test_state_command_and_overflow(self):
        self.assertEqual(self.events[10]["type"], "state")
        self.assertFalse(any(self.events[10][k] for k in ("up", "down", "left", "right")))
        self.assertEqual(self.events[-2]["type"], "error")
        self.assertEqual(self.events[-2]["code"], "event_overflow")
        self.assertEqual(self.events[-1]["type"], "state")
        self.assertTrue(self.events[-1]["up"])


class EncoderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lines, cls.events = build_events("test_encoder.cpp")

    def test_framing_and_startup(self):
        self.assertEqual([e["seq"] for e in self.events], list(range(len(self.events))))
        self.assertTrue(all(e["v"] == 0 for e in self.events))
        self.assertTrue(all(len(line.encode()) + 1 <= 256 for line in self.lines))
        self.assertEqual(self.events[0]["encoder"], {
            "clk": 32, "dt": 33, "sw": 23, "edges_per_step": 4, "rest_ab": 3, "inverted": True})
        self.assertEqual(self.events[1]["type"], "state")
        self.assertTrue(self.events[1]["button"])
        self.assertEqual(self.events[1]["ab"], 3)

    def test_ordered_steps_and_recovery(self):
        steps = [e for e in self.events if e["type"] == "encoder"]
        self.assertEqual([(e["direction"], e["delta"], e["ms"]) for e in steps],
                         [("ccw", -1, 100), ("cw", 1, 101), ("ccw", -1, 240)])

    def test_button_bounce_and_concurrent_joystick(self):
        buttons = [e for e in self.events if e["type"] == "button"]
        self.assertEqual([(e["pressed"], e["ms"]) for e in buttons], [(False, 150), (True, 170)])
        stick = [e for e in self.events if e["type"] == "joystick"]
        self.assertEqual(len(stick), 1)
        self.assertTrue(stick[0]["up"])
        self.assertFalse(any(stick[0][k] for k in ("down", "left", "right")))
        snapshot = next(e for e in self.events if e["type"] == "state" and e["ms"] == 180)
        self.assertTrue(snapshot["button"] and snapshot["up"])

    def test_edge_overflow_snapshot(self):
        errors = [i for i, e in enumerate(self.events) if e["type"] == "error"]
        self.assertEqual(len(errors), 1)
        self.assertEqual(self.events[errors[0]]["code"], "event_overflow")
        snapshot = self.events[errors[0] + 1]
        self.assertEqual(snapshot["type"], "state")
        self.assertTrue(snapshot["button"] and snapshot["up"])
        self.assertEqual(snapshot["ab"], 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
