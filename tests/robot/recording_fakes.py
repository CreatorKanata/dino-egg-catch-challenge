"""tests/robot/recording_fakes.py: Fake robot, clock, and dataset shared by the recorder tests.

Stand-ins for LeKiwiClient (observation with the six arm keys, recorded actions), a monotonic
clock that advances on sleep, and a LeRobotDataset with the calls the recorder makes. No
LeRobot import, no hardware.
"""

from robot.config import ARM_KEYS

CATCH = {key: float(value) for key, value in zip(ARM_KEYS, (-1.4, -104.8, 97.8, 58.3, -4.9, 1.6))}


class FakeRobot:
    """Mimics LeKiwiClient: get_observation, send_action, connect state, disconnect."""

    def __init__(self, arm=None, log=None):
        self.arm = dict(arm if arm is not None else CATCH)
        self.sent = []
        self.is_connected = True
        self.log = log if log is not None else []

    def get_observation(self):
        return {**self.arm, "x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}

    def send_action(self, action):
        self.sent.append(dict(action))
        self.log.append("send")

    def disconnect(self):
        self.is_connected = False
        self.log.append("robot.disconnect")


class FakeClock:
    """now() returns the fake time; sleep() advances it (at least `min_step` so loops progress)."""

    def __init__(self, start=0.0, min_step=1 / 30):
        self.time = start
        self.min_step = min_step

    def now(self):
        return self.time

    def sleep(self, seconds):
        self.time += max(seconds, self.min_step)


class FakeDataset:
    def __init__(self, log, pending=False, num_episodes=0, fail_finalize=False):
        self.log = log
        self.pending = pending
        self.num_episodes = num_episodes
        self.fail_finalize = fail_finalize
        self.repo_id = "me/data"

    def has_pending_frames(self):
        return self.pending

    def save_episode(self):
        self.log.append("save")
        self.pending = False
        self.num_episodes += 1

    def clear_episode_buffer(self):
        self.log.append("clear")
        self.pending = False

    def finalize(self):
        self.log.append("finalize")
        if self.fail_finalize:
            raise RuntimeError("disk full")

    def push_to_hub(self):
        self.log.append("push")
