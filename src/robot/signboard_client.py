"""src/robot/signboard_client.py: Manual Mode side of the out-of-process signboard.

Starts `python -m robot.signboard_process` as a separate interpreter (opencv and pygame each
bundle libSDL2 on macOS, so pygame must not load into the LeRobot process) and streams display
packets to its stdin from a daemon writer thread. Only the newest packet is kept, so the
control loop never blocks on the display. A daemon reader thread collects the KachiButton
command lines the child prints on stdout; poll_commands() drains them without blocking.
Deliberately not `multiprocessing`: its spawn start method re-imports the parent's main module
(and with it lerobot and cv2) in the child.
"""

from collections.abc import Mapping, Sequence
import logging
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
from typing import Any

from robot.config import SIGNBOARD_CHILD_EXIT_TIMEOUT_S, SIGNBOARD_FULLSCREEN, SIGNBOARD_SIZE
from robot.dino_controller_reader import ControllerState
from robot.display_status import DisplayStatus
from robot.drive_state import DriveState
from robot.signboard_protocol import (
    CLOSE_MESSAGE,
    COMMAND_LINE_MAX_BYTES,
    DisplayPacket,
    encode,
    parse_command_line,
)

logger = logging.getLogger(__name__)

SOURCE_ROOT = str(Path(__file__).resolve().parents[1])  # directory that contains `robot/`
TERMINATE_WAIT_S = 1.0


def child_env(base: Mapping[str, str] | None) -> dict[str, str]:
    """Environment for the child: `base` (default os.environ) with src/ first on PYTHONPATH."""
    env = dict(os.environ if base is None else base)
    existing = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = os.pathsep.join(part for part in (SOURCE_ROOT, existing) if part)
    return env


class SignboardClient:
    """DisplaySink backed by a child process: open, render, pump, poll_commands, close."""

    def __init__(
        self,
        fullscreen: bool = SIGNBOARD_FULLSCREEN,
        size: tuple[int, int] = SIGNBOARD_SIZE,
        command: Sequence[str] | None = None,
        env: Mapping[str, str] | None = None,
        exit_timeout_s: float = SIGNBOARD_CHILD_EXIT_TIMEOUT_S,
    ) -> None:
        flags = ["--size", f"{size[0]}x{size[1]}", *(["--fullscreen"] if fullscreen else [])]
        default = [sys.executable, "-m", "robot.signboard_process", *flags]
        self._command = tuple(default if command is None else command)
        self._env = child_env(env)
        self._exit_timeout_s = exit_timeout_s
        self._condition = threading.Condition()
        self._pending: bytes | None = None
        self._stopping = False
        self._alive = False
        self._process: subprocess.Popen | None = None
        self._writer: threading.Thread | None = None
        self._reader: threading.Thread | None = None
        self._commands: queue.Queue[str] = queue.Queue()
        self.returncode: int | None = None

    def open(self) -> None:
        self._process = subprocess.Popen(
            self._command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None, env=self._env
        )
        self._alive = True
        self._writer = threading.Thread(target=self._write_loop, name="signboard-writer", daemon=True)
        self._writer.start()
        self._reader = threading.Thread(target=self._read_loop, name="signboard-reader", daemon=True)
        self._reader.start()
        logger.info("Signboard process started (pid %d)", self._process.pid)

    def render(
        self,
        frames: Mapping[str, Any],
        drive: DriveState,
        controller: ControllerState,
        status: DisplayStatus = DisplayStatus(),
    ) -> None:
        """Encode now (cheap copies) and hand the newest packet to the writer; never blocks on I/O."""
        if not self.pump():
            return
        packet = DisplayPacket(drive=drive, controller=controller, frames=tuple(frames.items()), status=status)
        data = encode(packet)
        with self._condition:
            self._pending = data  # replaces any packet the writer has not taken yet
            self._condition.notify()

    def pump(self) -> bool:
        """True while the child runs and its pipe works; False after ESC, close, or a crash."""
        return self._process is not None and self._process.poll() is None and self._alive

    def poll_commands(self) -> tuple[str, ...]:
        """KachiButton commands received since the last call, oldest first; never blocks."""
        commands: list[str] = []
        while True:
            try:
                commands.append(self._commands.get_nowait())
            except queue.Empty:
                return tuple(commands)

    def close(self) -> None:
        """Ask the child to exit, then escalate to terminate and kill. Safe to call repeatedly."""
        process = self._process
        if process is None:
            return
        with self._condition:
            self._pending = CLOSE_MESSAGE
            self._stopping = True
            self._condition.notify()
        self._join_writer(process)
        self._close_stdin(process)
        self.returncode = self._wait_for_exit(process)
        self._join_reader(process)
        self._process = None
        self._alive = False
        logger.info("Signboard process exited with code %s", self.returncode)

    def _write_loop(self) -> None:
        stdin = self._process.stdin if self._process is not None else None
        while stdin is not None:
            with self._condition:
                while self._pending is None and not self._stopping:
                    self._condition.wait()
                data, self._pending = self._pending, None
            if data is None:
                return
            try:
                stdin.write(data)
                stdin.flush()
            except (BrokenPipeError, OSError, ValueError) as error:
                logger.warning("Signboard pipe closed: %s", error)
                self._alive = False
                return
            if data is CLOSE_MESSAGE:
                return

    def _read_loop(self) -> None:
        """Queue every valid command line from the child's stdout; ends at EOF (child exit)."""
        stdout = self._process.stdout if self._process is not None else None
        if stdout is None:
            return
        try:
            for line in iter(lambda: stdout.readline(COMMAND_LINE_MAX_BYTES), b""):
                command = parse_command_line(line)
                if command is None:
                    logger.debug("Dropping malformed signboard output: %r", line)
                else:
                    self._commands.put(command)
        except (OSError, ValueError) as error:
            logger.debug("Signboard output closed: %s", error)

    def _join_reader(self, process: subprocess.Popen) -> None:
        if self._reader is not None:
            self._reader.join(self._exit_timeout_s)  # the child has exited, so EOF is imminent
            if self._reader.is_alive():
                logger.warning("Signboard reader did not finish")
        try:
            if process.stdout is not None:
                process.stdout.close()
        except OSError:
            pass  # already closed

    def _join_writer(self, process: subprocess.Popen) -> None:
        if self._writer is None:
            return
        self._writer.join(self._exit_timeout_s)
        if self._writer.is_alive():  # blocked on a full pipe: unblock it by ending the child
            logger.warning("Signboard writer is stuck; terminating the signboard process")
            process.terminate()
            self._writer.join(TERMINATE_WAIT_S)

    @staticmethod
    def _close_stdin(process: subprocess.Popen) -> None:
        try:
            if process.stdin is not None:
                process.stdin.close()
        except (BrokenPipeError, OSError, ValueError):
            pass  # the child is already gone; nothing left to flush

    def _wait_for_exit(self, process: subprocess.Popen) -> int:
        try:
            return process.wait(self._exit_timeout_s)
        except subprocess.TimeoutExpired:
            logger.warning("Signboard process did not exit; terminating")
            process.terminate()
        try:
            return process.wait(TERMINATE_WAIT_S)
        except subprocess.TimeoutExpired:
            logger.warning("Signboard process did not terminate; killing")
            process.kill()
            return process.wait()
