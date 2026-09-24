"""src/robot/display_status.py: What the signboard shows about modes and the arm, as plain data.

A frozen DisplayStatus travels from the control loop to the signboard child over the pipe
(signboard_protocol.py) and is turned into status text by signboard_layout.py. Stdlib-only, so
both processes can import it (the child must never load LeRobot or cv2).
"""

from dataclasses import dataclass
from typing import Final, Literal

from robot.mode_manager import Action, AppState, Mode

ArmStatus = Literal["holding", "syncing", "following", "leader fault", "no leader"]
ARM_STATUSES: Final = ("holding", "syncing", "following", "leader fault", "no leader")


@dataclass(frozen=True)
class DisplayStatus:
    """Mode, running action, FSC voice input, current notice, arm follow status, Stop latch."""

    mode: Mode = "manual"
    action: Action = "none"
    voice_listening: bool = False
    notice: str = ""
    arm_status: ArmStatus = "holding"
    stopped: bool = False


def display_status(app: AppState, arm_status: ArmStatus) -> DisplayStatus:
    """The displayable part of the application state plus the arm status."""
    return DisplayStatus(
        mode=app.mode,
        action=app.action,
        voice_listening=app.voice_listening,
        notice=app.notice,
        arm_status=arm_status,
        stopped=app.stopped,
    )
