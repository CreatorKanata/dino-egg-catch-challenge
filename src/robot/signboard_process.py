"""src/robot/signboard_process.py: Signboard child process (`python -m robot.signboard_process`).

Reads display packets from stdin and draws them with SignboardView; KachiButton text typed
into the window is matched by kachi_phrases and each command is written to stdout as one JSON
line. It runs in its own interpreter because opencv (pulled in by LeRobot) and pygame each
bundle libSDL2, and both copies cannot safely live in one macOS process. Therefore this module
must NEVER import lerobot or cv2 (directly or through robot.drive_loop, robot.top_camera,
robot.lekiwi_adapter, robot.leader_arm, or robot.teleop_drive). Only stdlib, numpy, pygame (via
robot.signboard), the protocol, the pure phrase matcher, and config are allowed. Logs go to
stderr; stdout carries command lines only.
"""

import argparse
import logging
import os
import select
import sys
import time
from collections.abc import Callable
from typing import BinaryIO

from robot.config import SIGNBOARD_CHILD_POLL_S, SIGNBOARD_FULLSCREEN, SIGNBOARD_SIZE
from robot.kachi_phrases import PhraseBuffer, feed
from robot.signboard import SignboardView
from robot.signboard_protocol import CloseRequest, encode_command, read_packet

logger = logging.getLogger("robot.signboard_process")


def parse_size(text: str) -> tuple[int, int]:
    """Parse `WxH` into a (width, height) tuple of positive integers."""
    try:
        width, height = (int(part) for part in text.lower().split("x"))
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"size must look like 1280x720, got {text!r}") from error
    if width <= 0 or height <= 0:
        raise argparse.ArgumentTypeError(f"size must be positive, got {text!r}")
    return width, height


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Dino Egg Catch Challenge signboard (reads packets on stdin).")
    parser.add_argument("--fullscreen", action="store_true", default=SIGNBOARD_FULLSCREEN,
                        help="show the signboard fullscreen")
    default_size = f"{SIGNBOARD_SIZE[0]}x{SIGNBOARD_SIZE[1]}"
    parser.add_argument("--size", type=parse_size, default=SIGNBOARD_SIZE,
                        help=f"window size as WxH (default: {default_size})")
    return parser.parse_args(argv)


def emit_commands(stdout: BinaryIO, commands: tuple[str, ...]) -> None:
    """Write one JSON line per command and flush, so the parent sees it immediately."""
    for command in commands:
        logger.info("KachiButton command: %s", command)
        stdout.write(encode_command(command))
    if commands:
        stdout.flush()


def serve(
    view: SignboardView,
    stdin: BinaryIO,
    stdout: BinaryIO,
    clock: Callable[[], float] = time.monotonic,
) -> int:
    """Render packets and report KachiButton commands until EOF, a close request, ESC, or close."""
    phrases = PhraseBuffer()
    while True:
        readable, _, _ = select.select([stdin], [], [], SIGNBOARD_CHILD_POLL_S)
        if readable:
            packet = read_packet(stdin)
            if packet is None or isinstance(packet, CloseRequest):
                logger.info("Signboard input ended (%s)", "close request" if packet else "EOF or malformed")
                return 0
            view.render(dict(packet.frames), packet.drive, packet.controller, packet.status)
        result = view.pump()
        if result.typed:
            phrases, commands = feed(phrases, result.typed, clock())
            emit_commands(stdout, commands)
        if not result.keep_running:
            logger.info("Signboard closed by ESC or window close")
            return 0


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(stream=sys.stderr, level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    # Unbuffered stdin: with a buffered reader, select() cannot see packets already read ahead
    # into the Python buffer, and the display would lag one packet behind.
    stdin = os.fdopen(sys.stdin.fileno(), "rb", buffering=0, closefd=False)
    view = SignboardView(size=args.size, fullscreen=args.fullscreen)
    try:
        view.open()
        return serve(view, stdin, sys.stdout.buffer)
    except KeyboardInterrupt:
        logger.info("Signboard stopped by Ctrl+C")
        return 0
    finally:
        view.close()


if __name__ == "__main__":
    sys.exit(main())
