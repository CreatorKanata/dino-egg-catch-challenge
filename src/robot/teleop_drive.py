"""src/robot/teleop_drive.py: Manual Mode entry point (`python -m robot.teleop_drive`).

Opens the dino-controller, the LeKiwi connection, the leader arm, the overhead camera, the
attendee signboard (a separate interpreter that also receives KachiButton phrases, see
signboard_client.py), and optionally Rerun; runs the loop in drive_loop.py; and guarantees
zero base velocities before anything is disconnected. Mode rules live in mode_manager.py,
per-frame decisions in manual_mode.py, and the drive rules in drive_state.py.
"""

import argparse
import logging
from collections.abc import Callable
from typing import Any

from lerobot.utils.visualization_utils import init_rerun

from robot.config import (
    CONTROLLER_SERIAL_PORT,
    LEADER_ARM_PORT,
    LOOP_HZ,
    PI_REMOTE_IP,
    RERUN_SESSION_NAME,
    SIGNBOARD_FULLSCREEN,
)
from robot.dino_controller_reader import SerialControllerReader
from robot.drive_loop import DriveDevices, loop
from robot.leader_arm import LeaderArm
from robot.lekiwi_adapter import LeKiwiAdapter
from robot.signboard_client import SignboardClient
from robot.top_camera import TopCamera

logger = logging.getLogger("robot.teleop_drive")

HOST_COMMAND = "python -m lerobot.robots.lekiwi.lekiwi_host --robot.id=dino_kiwi"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Manual Mode: drive the LeKiwi base with the dino-controller and move the arm with the leader arm."
    )
    parser.add_argument("--no-camera", action="store_true", help="do not open the overhead camera")
    parser.add_argument("--no-signboard", action="store_true", help="do not open the attendee signboard window")
    parser.add_argument("--fullscreen", action="store_true", default=SIGNBOARD_FULLSCREEN,
                        help="show the signboard fullscreen (overrides config)")
    parser.add_argument("--rerun", action="store_true", help="also start the Rerun operator view")
    parser.add_argument("--controller-port", default=CONTROLLER_SERIAL_PORT,
                        help=f"dino-controller serial port (default: {CONTROLLER_SERIAL_PORT})")
    parser.add_argument("--remote-ip", default=PI_REMOTE_IP,
                        help=f"LeKiwi host address (default: {PI_REMOTE_IP})")
    parser.add_argument("--leader-port", default=LEADER_ARM_PORT,
                        help=f"leader arm serial port (default: {LEADER_ARM_PORT})")
    parser.add_argument("--no-leader", action="store_true",
                        help="do not open the leader arm; the arm holds its pose (drive-only Manual Mode)")
    return parser.parse_args(argv)


def build_devices(args: argparse.Namespace) -> DriveDevices:
    return DriveDevices(
        reader=SerialControllerReader(args.controller_port),
        adapter=LeKiwiAdapter(remote_ip=args.remote_ip),
        camera=None if args.no_camera else TopCamera(),
        view=None if args.no_signboard else SignboardClient(fullscreen=args.fullscreen),
        use_rerun=args.rerun,
        leader=None if args.no_leader else LeaderArm(port=args.leader_port),
    )


def connect(devices: DriveDevices) -> None:
    """Open devices in order: controller, robot (captures the arm pose), leader, camera, signboard, Rerun."""
    devices.reader.open()
    devices.adapter.connect()
    if devices.leader is not None:
        devices.leader.connect()
    if devices.camera is not None:
        devices.camera.connect()
    if devices.view is not None:
        devices.view.open()
    if devices.use_rerun:
        init_rerun(session_name=RERUN_SESSION_NAME)


def safely(label: str, action: Callable[[], Any]) -> None:
    """Run one shutdown step so a failure cannot skip the remaining steps."""
    try:
        action()
    except Exception:
        logger.exception("Shutdown step failed: %s", label)


def shutdown(devices: DriveDevices) -> None:
    """Zero velocities first, then disconnect robot, leader, camera, controller, and signboard."""
    safely("stop base", devices.adapter.stop)
    safely("disconnect robot", devices.adapter.disconnect)
    if devices.leader is not None:
        safely("disconnect leader arm", devices.leader.disconnect)
    if devices.camera is not None:
        safely("disconnect camera", devices.camera.disconnect)
    safely("close controller", devices.reader.close)
    if devices.view is not None:
        safely("close signboard", devices.view.close)


def run(args: argparse.Namespace) -> None:
    devices = build_devices(args)
    try:
        connect(devices)
        logger.info("Manual Mode running at %d Hz; press Ctrl+C (or ESC in the signboard) to quit", LOOP_HZ)
        loop(devices)
        logger.info("Stopping Manual Mode: signboard closed or exited")
    except KeyboardInterrupt:
        logger.info("Stopping Manual Mode: Ctrl+C")
    except Exception:
        logger.exception("Stopping Manual Mode: unexpected error")
        raise
    finally:
        shutdown(devices)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    print(f"Start the host on the Pi first: {HOST_COMMAND}")
    leader = "none (--no-leader)" if args.no_leader else args.leader_port
    print(f"Controller: {args.controller_port}  Robot: {args.remote_ip}  Leader arm: {leader}")
    run(args)


if __name__ == "__main__":
    main()
