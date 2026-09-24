"""src/robot/signboard_process.py: Signboard child process (`python -m robot.signboard_process`).

Reads display packets from stdin and draws them with SignboardView. It runs in its own
interpreter because opencv (pulled in by LeRobot) and pygame each bundle libSDL2, and both
copies cannot safely live in one macOS process. Therefore this module must NEVER import
lerobot or cv2 (directly or through robot.drive_loop, robot.top_camera, robot.lekiwi_adapter,
or robot.teleop_drive). Only stdlib, numpy, pygame (via robot.signboard), the protocol, and
config are allowed. Logs go to stderr; stdout stays unused.
"""

import argparse
import logging
import os
import select
import sys
from typing import BinaryIO

from robot.config import SIGNBOARD_CHILD_POLL_S, SIGNBOARD_FULLSCREEN, SIGNBOARD_SIZE
from robot.signboard import SignboardView
from robot.signboard_protocol import CloseRequest, read_packet

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


def serve(view: SignboardView, stdin: BinaryIO) -> int:
    """Render packets until EOF, a close request, ESC, or the window close button."""
    while True:
        readable, _, _ = select.select([stdin], [], [], SIGNBOARD_CHILD_POLL_S)
        if readable:
            packet = read_packet(stdin)
            if packet is None or isinstance(packet, CloseRequest):
                logger.info("Signboard input ended (%s)", "close request" if packet else "EOF or malformed")
                return 0
            view.render(dict(packet.frames), packet.drive, packet.controller)
        if not view.pump():
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
        return serve(view, stdin)
    except KeyboardInterrupt:
        logger.info("Signboard stopped by Ctrl+C")
        return 0
    finally:
        view.close()


if __name__ == "__main__":
    sys.exit(main())
