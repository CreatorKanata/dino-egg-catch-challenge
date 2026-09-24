"""src/robot/arm_motions.py: Home pose and recorded release motion files for Auto Release.

The home pose (neck folded, mouth down) and the release motion (a fixed joint trajectory recorded
from the leader arm, not a learned policy) are small JSON files under data/arm/ (HOME_POSE_PATH and
RELEASE_MOTION_PATH in config.py; formats in src/robot/README.md). This module validates them
(exactly the six ARM_KEYS as finite numbers, strictly increasing t, at least two frames), resamples
a motion by linear interpolation, time-scales it for slower playback, and writes new files
atomically (the previous motion is kept as `<name>.prev.json`). Pure except the explicit load and
save functions; stdlib-only.
"""

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import shutil
from typing import Any, Final

from robot.config import ARM_KEYS, LOOP_HZ, RELEASE_PLAYBACK_SPEED

FILE_VERSION: Final = 1
MIN_FRAMES: Final = 2

Pose = Mapping[str, float]


class ArmFileError(ValueError):
    """A home pose or release motion file that is not valid."""


@dataclass(frozen=True)
class MotionFrame:
    """One recorded frame: seconds since the recording started and the commanded arm pose."""

    t: float
    pose: Pose


@dataclass(frozen=True)
class ArmMotion:
    """A recorded joint trajectory: recording rate and at least two frames with increasing t."""

    rate_hz: float
    frames: tuple[MotionFrame, ...]

    @property
    def duration_s(self) -> float:
        return self.frames[-1].t - self.frames[0].t


def _number(value: Any, what: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ArmFileError(f"{what} must be a finite number, got {value!r}")
    return float(value)


def parse_pose(fields: Any, what: str = "pose") -> dict[str, float]:
    """Exactly the six ARM_KEYS with finite numeric values -> a new pose dict."""
    if not isinstance(fields, dict) or set(fields) != set(ARM_KEYS):
        raise ArmFileError(f"{what} must have exactly the keys {', '.join(ARM_KEYS)}")
    return {key: _number(fields[key], f"{what}[{key}]") for key in ARM_KEYS}


def _record(data: Any) -> dict[str, Any]:
    if not isinstance(data, dict) or data.get("version") != FILE_VERSION:
        raise ArmFileError(f"expected a JSON object with \"version\": {FILE_VERSION}")
    return data


def parse_home(data: Any) -> dict[str, float]:
    """The home pose from a parsed home_pose.json document."""
    return parse_pose(_record(data).get("pose"), "pose")


def parse_motion(data: Any) -> ArmMotion:
    """The motion from a parsed release_motion.json document (>= 2 frames, strictly increasing t)."""
    record = _record(data)
    rate_hz = _number(record.get("rate_hz"), "rate_hz")
    entries = record.get("frames")
    if rate_hz <= 0 or not isinstance(entries, list) or len(entries) < MIN_FRAMES:
        raise ArmFileError(f"need rate_hz > 0 and a list of at least {MIN_FRAMES} frames")
    frames = tuple(_frame(entry, index) for index, entry in enumerate(entries))
    if any(later.t <= earlier.t for earlier, later in zip(frames, frames[1:])):
        raise ArmFileError("frame times must increase strictly")
    return ArmMotion(rate_hz=rate_hz, frames=frames)


def _frame(entry: Any, index: int) -> MotionFrame:
    if not isinstance(entry, dict):
        raise ArmFileError(f"frames[{index}] must be an object")
    return MotionFrame(t=_number(entry.get("t"), f"frames[{index}].t"), pose=parse_pose(entry.get("pose"),
                                                                                         f"frames[{index}].pose"))


def _read(path: Path) -> Any:
    """The parsed JSON document. Raises OSError (missing, unreadable) or ArmFileError."""
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ArmFileError(f"{path}: not valid JSON ({error})") from error


def load_pose(path: Path) -> dict[str, float]:
    """Load and validate home_pose.json. Raises OSError or ArmFileError."""
    return parse_home(_read(path))


def load_motion(path: Path) -> ArmMotion:
    """Load and validate release_motion.json. Raises OSError or ArmFileError."""
    return parse_motion(_read(path))


def _lerp(first: Pose, second: Pose, fraction: float) -> dict[str, float]:
    return {key: first[key] + (second[key] - first[key]) * fraction for key in ARM_KEYS}


def resample(motion: ArmMotion, dt: float) -> tuple[dict[str, float], ...]:
    """Poses every `dt` seconds from the first frame, linearly interpolated; the last pose is
    always the last frame."""
    if not dt > 0:
        raise ValueError("dt must be positive")
    frames, start = motion.frames, motion.frames[0].t
    steps = int(math.floor(motion.duration_s / dt + 1e-9))
    poses, segment = [], 0
    for step in range(steps + 1):
        t = start + step * dt
        while segment < len(frames) - 2 and frames[segment + 1].t < t:
            segment += 1
        first, second = frames[segment], frames[segment + 1]
        fraction = min(1.0, max(0.0, (t - first.t) / (second.t - first.t)))
        poses.append(_lerp(first.pose, second.pose, fraction))
    last = dict(frames[-1].pose)
    return tuple(poses) if poses[-1] == last else tuple(poses) + (last,)


def time_scale(motion: ArmMotion, factor: float) -> ArmMotion:
    """The motion played at `factor` times the recorded speed (0.5 takes twice as long)."""
    if not factor > 0:
        raise ValueError("factor must be positive")
    frames = tuple(MotionFrame(t=frame.t / factor, pose=frame.pose) for frame in motion.frames)
    return ArmMotion(rate_hz=motion.rate_hz, frames=frames)


def playback_frames(
    motion: ArmMotion, speed: float = RELEASE_PLAYBACK_SPEED, rate_hz: float = LOOP_HZ
) -> tuple[dict[str, float], ...]:
    """One pose per loop frame for playback at `speed` times the recorded speed."""
    return resample(time_scale(motion, speed), 1.0 / rate_hz)


def home_record(pose: Pose, recorded_at: str, note: str) -> dict[str, Any]:
    """The home_pose.json document for `pose`."""
    return {"version": FILE_VERSION, "recorded_at": recorded_at, "note": note, "pose": parse_pose(dict(pose))}


def motion_record(frames: Sequence[MotionFrame], rate_hz: float, recorded_at: str, note: str) -> dict[str, Any]:
    """The release_motion.json document; validated like a loaded file (ArmFileError otherwise)."""
    record = {"version": FILE_VERSION, "recorded_at": recorded_at, "note": note, "rate_hz": rate_hz,
              "frames": [{"t": round(frame.t, 4), "pose": dict(frame.pose)} for frame in frames]}
    parse_motion(record)
    return record


def write_record(path: Path, record: Mapping[str, Any], backup: bool = False) -> None:
    """Write the JSON document atomically (temporary file, then rename). With `backup`, an existing
    file is first copied to `<stem>.prev.json` (one backup). Raises OSError."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    if backup and target.exists():
        shutil.copyfile(target, target.with_name(f"{target.stem}.prev.json"))
    temporary = target.with_name(f".{target.name}.tmp")
    temporary.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, target)
