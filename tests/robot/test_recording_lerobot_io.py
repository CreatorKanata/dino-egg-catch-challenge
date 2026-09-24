"""tests/robot/test_recording_lerobot_io.py: Checks of the recorder's LeRobot wiring with fake lerobot modules.

Fake modules stand in for the fork's packages in sys.modules, so these tests verify that
record_loop is called as examples/lekiwi/record.py does (teleop=[leader, keyboard], dataset only
while recording, the task with the egg color), that the dataset is created or resumed with the
robot's features, and that record() wires start-up, session, and shutdown. The real LeRobot is
never imported here, and no hardware is opened.
"""

from argparse import Namespace
import io
from contextlib import redirect_stdout
from pathlib import Path
import sys
from types import ModuleType
import unittest
from unittest import mock

from robot.recording import lerobot_io, record_pick_egg
from robot.recording.record_pick_egg import parse_args
from robot.recording.session import SessionPlan, plan_from_args
from robot.recording.speech import Speaker

from recording_fakes import CATCH, FakeDataset, FakeRobot

PLAN = SessionPlan("me/data", 2, 20.0, 15.0, 30, "Pick up the egg with the mouth", "red", True)


def module(name, **attributes):
    fake = ModuleType(name)
    for key, value in attributes.items():
        setattr(fake, key, value)
    return fake


def fake_lerobot(**modules):
    """Patch sys.modules with `lerobot` plus the given dotted submodules (attribute dicts)."""
    entries = {"lerobot": module("lerobot")}
    entries.update({name: module(name, **attrs) for name, attrs in modules.items()})
    return mock.patch.dict(sys.modules, entries)


class FakeSpeaker:
    def __init__(self, spoken, idle=True):
        self.spoken, self.idle = spoken, idle

    def say(self, text):
        self.spoken.append(text)

    def say_replaceable(self, text):
        self.spoken.append(("gate", text))

    def wait_idle(self, timeout_s):
        self.spoken.append(("wait", timeout_s))
        return self.idle


class SessionIOTests(unittest.TestCase):
    def make_io(self, calls, spoken, idle=True):
        record_loop = mock.Mock(side_effect=lambda **kwargs: calls.append(kwargs))
        with fake_lerobot(**{"lerobot.processor": {"make_default_processors": lambda: ("t", "r", "o")},
                             "lerobot.scripts": {}, "lerobot.scripts.lerobot_record": {"record_loop": record_loop}}):
            robot, dataset = FakeRobot(), FakeDataset([], pending=True)
            events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}
            pose = {key.removeprefix("arm_"): value for key, value in CATCH.items()}
            leader = mock.Mock(get_action=lambda: pose)
            session_io = lerobot_io.make_session_io(PLAN, robot, leader, "keyboard", events, dataset, CATCH,
                                                    display=True, speaker=FakeSpeaker(spoken, idle))
        return session_io, robot, leader, dataset

    def test_record_and_reset_call_record_loop_like_the_fork_example(self):
        calls, spoken = [], []
        session_io, robot, leader, dataset = self.make_io(calls, spoken)
        session_io.record_episode()
        session_io.reset()
        recorded, reset = calls
        self.assertEqual(recorded["teleop"], [leader, "keyboard"])
        self.assertIs(recorded["dataset"], dataset)
        self.assertIs(recorded["robot"], robot)
        self.assertEqual((recorded["fps"], recorded["control_time_s"], recorded["display_data"]), (30, 20.0, True))
        self.assertEqual(recorded["single_task"], "Pick up the egg with the mouth (red egg)")
        self.assertEqual((recorded["teleop_action_processor"], recorded["robot_action_processor"],
                          recorded["robot_observation_processor"]), ("t", "r", "o"))
        self.assertIsNone(reset["dataset"])
        self.assertEqual(reset["control_time_s"], 15.0)

    def test_gate_save_discard_zero_and_say(self):
        calls, spoken = [], []
        session_io, robot, _, dataset = self.make_io(calls, spoken)
        self.assertEqual(session_io.gate(), "ready")  # the robot and leader already sit at the catch pose
        session_io.say("hello")
        session_io.wait_quiet()
        self.assertEqual(spoken, ["hello", ("wait", 3.0)])
        session_io.save_episode()
        session_io.discard_episode()
        self.assertEqual(dataset.log, ["save", "clear"])
        session_io.zero_base()
        self.assertEqual(robot.sent[-1]["x.vel"], 0.0)


    def test_gate_guidance_is_replaceable_and_a_busy_speaker_is_logged(self):
        spoken = []
        with mock.patch.object(lerobot_io, "run_gate", side_effect=lambda *args: args[5]("hint") or "timeout"):
            session_io, _, _, _ = self.make_io([], spoken, idle=False)
            self.assertEqual(session_io.gate(), "timeout")
        self.assertEqual(spoken, [("gate", "hint")])
        with self.assertLogs("robot.recording.lerobot_io", "INFO"):
            session_io.wait_quiet()


class DatasetAndDeviceTests(unittest.TestCase):
    def dataset_modules(self, dataset_class, sanity):
        features = lambda hw, prefix: {f"{prefix}.{name}": hw[name] for name in hw}  # noqa: E731
        return fake_lerobot(**{"lerobot.common": {}, "lerobot.common.control_utils": {
                                   "sanity_check_dataset_robot_compatibility": sanity},
                               "lerobot.datasets": {"LeRobotDataset": dataset_class},
                               "lerobot.utils": {}, "lerobot.utils.constants": {"ACTION": "action", "OBS_STR": "observation",
                                                                                "HF_LEROBOT_HOME": "/home/lr"},
                               "lerobot.utils.feature_utils": {"hw_to_dataset_features": features}})

    def robot(self):
        robot = mock.Mock(action_features={"a": float}, observation_features={"o": float})
        robot.name = "lekiwi_client"  # Mock(name=...) would name the mock instead
        return robot

    def test_create_uses_the_robot_features(self):
        dataset_class, sanity = mock.Mock(), mock.Mock()
        with self.dataset_modules(dataset_class, sanity):
            lerobot_io.open_dataset(self.robot(), PLAN, Path("/root"), resume=False)
            self.assertEqual(lerobot_io.lerobot_home(), Path("/home/lr"))
        kwargs = dataset_class.create.call_args.kwargs
        self.assertEqual(kwargs["features"], {"action.a": float, "observation.o": float})
        self.assertEqual((kwargs["repo_id"], kwargs["fps"], kwargs["root"], kwargs["robot_type"], kwargs["use_videos"]),
                         ("me/data", 30, Path("/root"), "lekiwi_client", True))
        sanity.assert_not_called()

    def test_resume_checks_compatibility(self):
        dataset_class, sanity = mock.Mock(), mock.Mock()
        with self.dataset_modules(dataset_class, sanity):
            dataset = lerobot_io.open_dataset(self.robot(), PLAN, Path("/root"), resume=True)
        self.assertEqual(dataset_class.resume.call_args.kwargs["root"], Path("/root"))
        sanity.assert_called_once()
        self.assertIs(sanity.call_args.args[0], dataset)

    def test_devices_input_and_rerun(self):
        made, rerun = [], mock.Mock()
        factory = lambda label: (lambda *args, **kwargs: made.append((label, args, kwargs)) or label)  # noqa: E731
        with fake_lerobot(**{"lerobot.robots": {}, "lerobot.robots.lekiwi": {"LeKiwiClient": factory("robot"),
                                                                            "LeKiwiClientConfig": dict},
                             "lerobot.teleoperators": {},
                             "lerobot.teleoperators.keyboard": {"KeyboardTeleop": factory("keyboard"),
                                                                "KeyboardTeleopConfig": dict},
                             "lerobot.teleoperators.so_leader": {"SO100Leader": factory("leader"),
                                                                 "SO100LeaderConfig": dict},
                             "lerobot.utils": {}, "lerobot.utils.keyboard_input": {
                                 "init_keyboard_listener": lambda: ("listener", {"stop_recording": False})},
                             "lerobot.utils.visualization_utils": {"init_rerun": rerun, "shutdown_rerun": rerun}}):
            self.assertEqual(lerobot_io.make_devices("1.2.3.4", "/dev/x"), ("robot", "leader", "keyboard"))
            self.assertEqual(lerobot_io.start_input(display=True)[0], "listener")
            lerobot_io.shutdown(None, None, None, None, None, push=True, display=True)
        self.assertEqual(made[0][1][0], {"remote_ip": "1.2.3.4", "id": "dino_kiwi"})
        self.assertEqual(made[1][1][0], {"port": "/dev/x", "id": "dino_leader_arm"})
        self.assertEqual(rerun.call_count, 2)


class RecordTests(unittest.TestCase):
    def test_record_connects_runs_and_shuts_down(self):
        args = parse_args(["--egg-color", "red", "--no-rerun", "--root", "/nonexistent/dino"])
        devices = [mock.Mock(name=name) for name in ("robot", "leader", "keyboard")]
        order = mock.Mock()
        for name, device in zip(("robot", "leader", "keyboard"), devices):
            device.connect.side_effect = lambda name=name: order(f"{name}.connect")
        patches = {"lerobot_home": Path("/unused"), "make_devices": tuple(devices), "open_dataset": "dataset",
                   "start_input": ("listener", {"stop_recording": False}), "make_session_io": "io", "shutdown": True}
        with mock.patch.multiple(lerobot_io, **{name: mock.Mock(return_value=value) for name, value in patches.items()}), \
                mock.patch.object(record_pick_egg, "load_pose", return_value=dict(CATCH)), \
                mock.patch("robot.recording.episode_loop.run_session") as run_session, \
                redirect_stdout(io.StringIO()) as out:
            run_session.return_value = mock.Mock(recorded=0, skipped=0, rerecorded=0, stopped=False, index=60,
                                                 durations_s=())
            self.assertEqual(record_pick_egg.record(args, plan_from_args(args), Speaker(None)), 0)
            shutdown_args = lerobot_io.shutdown.call_args.args
        self.assertEqual([c.args[0] for c in order.call_args_list], ["leader.connect", "keyboard.connect", "robot.connect"])
        self.assertEqual(shutdown_args, (*devices, "listener", "dataset", True, False))
        self.assertIn("Pushed to the Hub: yes", out.getvalue())

    def test_ctrl_c_during_startup_still_shuts_down(self):
        args = Namespace(**{**vars(parse_args(["--egg-color", "green"])), "root": "/nonexistent/dino"})
        devices = (mock.Mock(), mock.Mock(), mock.Mock())
        devices[1].connect.side_effect = KeyboardInterrupt
        with mock.patch.multiple(lerobot_io, lerobot_home=mock.Mock(return_value=Path("/unused")),
                                 make_devices=mock.Mock(return_value=devices), open_dataset=mock.Mock(return_value="ds"),
                                 shutdown=mock.Mock(return_value=False)), \
                mock.patch.object(record_pick_egg, "load_pose", return_value=dict(CATCH)), \
                redirect_stdout(io.StringIO()) as out, self.assertLogs("robot.recording.record_pick_egg", "WARNING"):
            self.assertEqual(record_pick_egg.record(args, plan_from_args(args), Speaker(None)), 0)
            lerobot_io.shutdown.assert_called_once()
        self.assertIn("Session stopped early", out.getvalue())


if __name__ == "__main__":
    unittest.main()
