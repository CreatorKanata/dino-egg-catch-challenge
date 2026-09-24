"""tests/robot/test_record_pick_egg.py: Checks of the recorder CLI, its import isolation, and its shutdown.

Argument parsing (defaults, overrides, required egg color), `--help` in a fresh interpreter with
LeRobot, `datasets`, and `av` made unimportable (the imports must stay lazy), no pygame / cv2 /
numpy / lerobot loaded by importing the recorder, the start-up refusals, and the shutdown order with
fake devices (zero base and disconnect first, then finalize and push). No hardware.
"""

import io
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from robot.config import LEADER_ARM_PORT, PI_REMOTE_IP
from robot.recording import lerobot_io, record_pick_egg
from robot.recording.record_pick_egg import main, parse_args

from recording_fakes import CATCH, FakeDataset, FakeRobot

SRC = str(Path(__file__).resolve().parents[2] / "src")
BLOCK = "import sys\nfor name in ('lerobot', 'datasets', 'av'):\n    sys.modules[name] = None\n"


def run_isolated(code):
    return subprocess.run([sys.executable, "-c", code], env={"PYTHONPATH": SRC, "PATH": "/usr/bin:/bin"},
                          capture_output=True, timeout=120)


class ParseTests(unittest.TestCase):
    def test_defaults(self):
        args = parse_args(["--egg-color", "green"])
        self.assertEqual((args.remote_ip, args.leader_port, args.root), (PI_REMOTE_IP, LEADER_ARM_PORT, None))
        self.assertFalse(args.no_push or args.resume or args.no_rerun)
        self.assertEqual(args.voice, "auto")

    def test_flags(self):
        args = parse_args(["--egg-color", "orange", "--resume", "--root", "/d", "--no-rerun", "--remote-ip", "1.2.3.4",
                           "--leader-port", "/dev/x"])
        self.assertEqual((args.egg_color, args.root, args.remote_ip, args.leader_port), ("orange", "/d", "1.2.3.4", "/dev/x"))
        self.assertTrue(args.resume and args.no_rerun)

    def test_egg_color_is_required_and_checked(self):
        for argv in ([], ["--egg-color", "blue"]):
            with self.subTest(argv=argv), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(argv)

    def test_invalid_plan_is_a_usage_error(self):
        with redirect_stderr(io.StringIO()) as err, self.assertRaises(SystemExit) as raised:
            main(["--egg-color", "red", "--num-episodes", "0"])
        self.assertEqual(raised.exception.code, 2)
        self.assertIn("--num-episodes", err.getvalue())


class IsolationTests(unittest.TestCase):
    def test_help_works_without_lerobot_datasets_or_av(self):
        result = run_isolated(BLOCK + "import runpy\nsys.argv = ['record_pick_egg', '--help']\n"
                              "runpy.run_module('robot.recording.record_pick_egg', run_name='__main__')\n")
        self.assertEqual(result.returncode, 0, result.stderr.decode())
        self.assertIn(b"--egg-color", result.stdout)
        self.assertNotIn(b"objc[", result.stderr)

    def test_recorder_imports_are_stdlib_only(self):
        result = run_isolated("import sys, robot.recording.record_pick_egg, robot.recording.lerobot_io\n"
                              "import robot.recording.session, robot.recording.episode_gate, robot.recording.gate_runner\n"
                              "import robot.recording.episode_loop, robot.recording.config_recording, robot.recording.speech\n"
                              "names = ('lerobot', 'pygame', 'cv2', 'numpy', 'datasets', 'av')\n"
                              "loaded = [m for m in names if m in sys.modules]\n"
                              "assert not loaded, loaded\n")
        self.assertEqual(result.returncode, 0, result.stderr.decode())


class StartupTests(unittest.TestCase):
    def run_main(self, argv):
        with redirect_stdout(io.StringIO()) as out, mock.patch("logging.basicConfig"), \
                self.assertLogs("robot.recording.record_pick_egg", "ERROR") as logs:
            code = main(argv)
        return code, out.getvalue(), "\n".join(logs.output)

    def test_missing_catch_pose_refuses(self):
        with mock.patch.object(record_pick_egg, "load_pose", side_effect=FileNotFoundError("nope")):
            code, out, logs = self.run_main(["--egg-color", "green", "--voice", "none"])
        self.assertEqual(code, 2)
        self.assertIn("Close the Manual Mode application", out)
        self.assertIn("Right arrow", out)
        self.assertIn("Voice: none (speech off", out)
        self.assertIn("Catch pose missing", logs)

    def test_existing_dataset_without_resume_refuses(self):
        with tempfile.TemporaryDirectory() as root, \
                mock.patch.object(record_pick_egg, "load_pose", return_value=dict(CATCH)), \
                mock.patch.object(lerobot_io, "lerobot_home", return_value=Path("/unused")):
            code, _, logs = self.run_main(["--egg-color", "green", "--root", root, "--voice", "none"])
        self.assertEqual(code, 2)
        self.assertIn("--resume", logs)


class FakeDevice:
    def __init__(self, name, log):
        self.name, self.log, self.is_connected = name, log, True

    def disconnect(self):
        self.is_connected = False
        self.log.append(f"{self.name}.disconnect")

    def stop(self):
        self.log.append(f"{self.name}.stop")


class ShutdownTests(unittest.TestCase):
    def devices(self, **dataset_options):
        log = []
        robot = FakeRobot(log=log)
        return log, robot, FakeDevice("leader", log), FakeDevice("keyboard", log), FakeDevice("listener", log), \
            FakeDataset(log, **dataset_options)

    def test_order_zero_base_first_then_finalize_and_push(self):
        log, robot, leader, keyboard, listener, dataset = self.devices(num_episodes=3)
        self.assertTrue(lerobot_io.shutdown(robot, leader, keyboard, listener, dataset, push=True, display=False))
        self.assertEqual(log, ["send", "robot.disconnect", "leader.disconnect", "keyboard.disconnect", "listener.stop",
                               "finalize", "push"])
        self.assertEqual([robot.sent[0][key] for key in ("x.vel", "y.vel", "theta.vel")], [0.0, 0.0, 0.0])

    def test_no_push_and_pending_frames_discarded(self):
        log, robot, leader, keyboard, listener, dataset = self.devices(num_episodes=3, pending=True)
        self.assertFalse(lerobot_io.shutdown(robot, leader, keyboard, listener, dataset, push=False, display=False))
        self.assertEqual(log[-2:], ["clear", "finalize"])

    def test_no_push_without_episodes_or_after_a_failed_finalize(self):
        for options in ({"num_episodes": 0}, {"num_episodes": 2, "fail_finalize": True}):
            log, robot, leader, keyboard, listener, dataset = self.devices(**options)
            with self.subTest(options=options), self.assertLogs("robot.recording.lerobot_io", "WARNING"):
                self.assertFalse(lerobot_io.shutdown(robot, leader, keyboard, None, dataset, push=True, display=False))
            self.assertNotIn("push", log)

    def test_disconnected_robot_and_no_dataset(self):
        log, robot, leader, keyboard, _, _ = self.devices()
        robot.is_connected = False
        self.assertFalse(lerobot_io.shutdown(robot, leader, keyboard, None, None, push=True, display=False))
        self.assertEqual(log, ["leader.disconnect", "keyboard.disconnect"])

    def test_save_skips_an_empty_episode(self):
        log = []
        with self.assertLogs("robot.recording.lerobot_io", "WARNING"):
            lerobot_io._save(FakeDataset(log, pending=False))
        lerobot_io._save(FakeDataset(log, pending=True))
        self.assertEqual(log, ["save"])


if __name__ == "__main__":
    unittest.main()
