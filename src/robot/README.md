<!-- src/robot/README.md: Explain the Drive Mode teleoperation package, how to run it, and what is verified. -->
# Robot Control

Robot integration for the Dino Egg Catch Challenge. The first operating mode implemented here is **Drive Mode**: an attendee drives the LeKiwi base with the dino-controller while a pygame signboard shows the overhead, front, and wrist cameras and the drive status to attendees; Rerun is an optional operator view. Puppet Mode and Autonomous Mode from the [concept](../../docs/concept.md) are not implemented yet.

Hardware adapters are kept separate from mode logic so behavior can be checked without moving a robot. Emergency stopping and communication-loss handling stay local (Mac application and Pi host), never behind a cloud service. Background and API references: [LeKiwi application development guide](../../docs/lekiwi-app-development.md) and [dino-controller protocol](../../docs/dino-controller-protocol.md).

## Modules

| File | Role | Hardware |
| --- | --- | --- |
| `config.py` | All tunables: Pi address, ZMQ ports, serial port, camera index, loop rate, speed levels, input roles, signboard size and theme | None |
| `dino_controller_reader.py` | Serial JSON v0 lines -> immutable `ControllerState`; framing, validation, synchronization, sequence gaps; periodic `STATE` requests | Serial port (pyserial, imported lazily) |
| `controller_to_action.py` | `ControllerState` -> `{x.vel, y.vel, theta.vel}`; speed-level stepping | None (stdlib only) |
| `drive_state.py` | Frozen `DriveState` (speed level, Catch request, input lost) and the stop/drive decision | None (stdlib only) |
| `top_camera.py` | `OpenCVCamera` wrapper for the overhead camera on the Mac (BGR, like the Pi frames decoded by the client) | USB camera |
| `lekiwi_adapter.py` | `LeKiwiClient` wrapper; captures the arm pose at connect, sends it with the base keys, zeros before disconnect | Network to the Pi |
| `signboard_layout.py` | Pure signboard layout (camera and status rectangles, letterboxing) and status text | None (stdlib only) |
| `signboard.py` | pygame signboard window: render cameras and status, report ESC/close (child process only) | Display |
| `signboard_protocol.py` | Pipe packet format: JSON header line + raw BGR frame bytes; close request | None (stdlib + numpy) |
| `signboard_process.py` | Signboard child entry point (`python -m robot.signboard_process`); never imports LeRobot or cv2 | Display |
| `signboard_client.py` | Drive Mode side: starts the child, streams the newest packet from a writer thread, reports liveness | Child process |
| `drive_loop.py` | One loop iteration and the 30 Hz loop; the display is any `DisplaySink` | All of the above |
| `teleop_drive.py` | Drive Mode entry point: CLI, connect in order, zeros-first shutdown | All of the above |

## Running Drive Mode

1. On the Raspberry Pi (`pi@lekiwi`, conda env `lerobot312`, inside `~/lerobot-dino-egg-catch-challenge`):

   ```bash
   python -m lerobot.robots.lekiwi.lekiwi_host --robot.id=dino_kiwi
   ```

2. On the Mac, from this repository root, use the conda env `lerobot312` (owner decision): `conda activate lerobot312`, or call `/opt/miniconda3/envs/lerobot312/bin/python` directly. The env has the LeRobot fork installed editable (`pip install -e ".[lekiwi,viz]"` in `../lerobot-dino-egg-catch-challenge`) plus pyserial and pygame-ce. Check that it imports the fork; the path must be under `lerobot-dino-egg-catch-challenge`:

   ```bash
   python -c "import lerobot; print(lerobot.__file__)"
   ```

   The root `pyproject.toml` describes the same dependencies for an alternative `uv` setup. Then start Drive Mode:

   ```bash
   PYTHONPATH=src python -m robot.teleop_drive
   PYTHONPATH=src python -m robot.teleop_drive --fullscreen --rerun
   PYTHONPATH=src python -m robot.teleop_drive --controller-port /dev/cu.usbserial-XXXX --no-camera --no-signboard
   ```

   Options: `--no-camera`, `--no-signboard`, `--fullscreen` (overrides `SIGNBOARD_FULLSCREEN`), `--rerun` (opt-in operator view), `--controller-port`, `--remote-ip`. Defaults come from `config.py`. Find the overhead camera index with `lerobot-find-cameras opencv` and set `TOP_CAMERA_INDEX`.

## Display

The attendee display is a pygame window (owner decision, 2026-09-24): the overhead camera large on the left, the front and wrist cameras stacked on the right, and a status bar with the mode (`DRIVE`, `CATCH!`, `INPUT LOST`), the speed level as footprints (fixed by config), and the held directions plus a rotation symbol (↺/↻, ASCII `CCW`/`CW`) while an encoder rotation is pending. A missing frame shows "no signal". Unicode symbols are used only when pygame's default font has them; otherwise ASCII fallbacks (`#`/`-`, `UP`/`RIGHT`) are shown.

The signboard runs as a separate interpreter (`python -m robot.signboard_process`, started by `SignboardClient`) because opencv (pulled in by LeRobot) and pygame each bundle their own `libSDL2` on macOS, and both copies cannot safely live in one process. Drive Mode sends it one packet per loop frame over a pipe; only the newest packet is kept, so the control loop never blocks on the display. The Drive Mode process never imports pygame and the signboard process never imports LeRobot or cv2; tests assert both.

The window is display only: no buttons and no keyboard control of the robot. ESC or the window close button (or the signboard process exiting for any reason) ends Drive Mode through the normal shutdown, so zero velocities are sent before disconnecting. The colors in `DEFAULT_THEME` are a placeholder Jurassic palette; artwork (wood-sign frames, fonts, footprint icons) will come later as image assets.

## Input mapping

| Controller input | Drive Mode action | Status |
| --- | --- | --- |
| Joystick up / down / left / right | Translate forward / backward / left / right (`x.vel`, `y.vel`; `LEFT_RIGHT_ROLE = "strafe"`) | Owner decision, 2026-09-24 |
| Rotary encoder | Rotate right (cw) / left (ccw); one click turns the base by half the knob's own angle per click (360° / detents × 0.5; currently assumed 20 detents = 9°), open-loop; the detent count needs the counted test. Queued rotation is capped at `ENCODER_MAX_PENDING_DEG` (360°, one knob turn) | Owner decision, 2026-09-24 |
| (none) | Speed level fixed at `INITIAL_SPEED_INDEX` (slow: 0.1 m/s, 30 deg/s); no input changes it | Owner decision, 2026-09-24 |
| Shaft button | Catch request: the base stops while it is held; no arm motion in this version | Default, owner decision pending |

Encoder clicks add a rotation budget in degrees; each frame the loop sends `theta.vel = ±theta` and subtracts what that speed covers in the measured frame time (clamped to 2 / `LOOP_HZ`). `ENCODER_DEGREES_PER_STEP` is derived as `360 / ENCODER_CLICKS_PER_REVOLUTION * ENCODER_ROTATION_SCALE` (scale 0.5, owner decision 2026-09-24). `ENCODER_CLICKS_PER_REVOLUTION = 20` is a placeholder until the detent count is measured (see "Detent calibration", still pending in [dino-controller-validation.md](../../docs/dino-controller-validation.md)). This is commanded, not measured, rotation, so the real heading change also depends on the wheels and floor. Rotating while translating is allowed. Contradictory directions cancel. Every command also carries the six arm positions captured once at connect (`ARM_MODE = "hold_initial_pose"`), because the fork's host fails on actions without arm keys (code reading, not tested on the robot). Arm values are never derived from controller input, so the arm is held at its startup pose. The arm mode and the shaft-button role are defaults chosen in `config.py`; those owner decisions are still pending (see the guide, section 5).

## Safety behavior

- **Stops clear pending rotation.** Input loss and Catch both reset the encoder rotation budget to zero, so rotation never resumes on its own after a stop.
- **Input lost -> zeros.** Until a valid `state` snapshot is received, after `ready`, `error`, or a sequence gap, and whenever no valid controller message arrives for `CONTROLLER_INPUT_TIMEOUT_S` (0.5 s), the loop sends zero velocities. The reader requests `STATE` every `CONTROLLER_STATE_POLL_INTERVAL_S` (0.2 s) because the firmware is silent while inputs are unchanged.
- **Arm held, never driven.** Drive Mode does not start unless the arm pose can be read at connect; every command, including stop commands, repeats that pose.
- **Ctrl+C, ESC/window close, or any loop error -> zeros, then disconnect.** Shutdown sends zero velocities before closing the robot connection, then closes the camera, controller, and signboard.
- **Host watchdog is the backstop.** The Pi host stops the base if no command arrives for 500 ms; the application does not rely on it.

## Tests

Hardware-free unit tests (no serial port, camera, or robot is opened):

```bash
PYTHONPATH=src /opt/miniconda3/envs/lerobot312/bin/python -m unittest discover -s tests/robot -v
```

Do not add `-t .`: the `lerobot312` environment has an unrelated `tests` package in `site-packages` that shadows this repository's `tests/` directory, so `tests.robot` cannot be imported. The invocation above matches the `tests/tactile_fingertip` suite.

## Verified vs. proposed

- **Verified without hardware:** the unit tests above pass (controller framing, parsing, synchronization, action mapping, the encoder rotation budget and its consumption, Catch and input-lost stops, arm-pose capture and nine-key action content with a fake client, signboard layout and status text, loop wiring with fake devices, headless pygame rendering with `SDL_VIDEODRIVER=dummy`, the pipe packet format, a headless round trip through a real signboard child process, and the import isolation of both processes), and the hardware modules import and `--help` runs in the `lerobot312` environment.
- **Owner-tested on the robot (2026-09-24):** the owner drove the LeKiwi base with the dino-controller through `robot.teleop_drive` and reported that the controller operation works. Not separately reported: the measured rotation per encoder click, the overhead camera, and the signboard in a real (non-headless) window. Each further physical run still needs the scope, reachable stop control, operating area, and verified limits agreed with the operator (see `AGENTS.md`).
- **Owner decisions (2026-09-24):** joystick translation, encoder rotation, fixed speed level. One click turns the base by half the knob's own angle per click (`ENCODER_ROTATION_SCALE = 0.5`); `ENCODER_CLICKS_PER_REVOLUTION` (20) is an unmeasured placeholder, and `ENCODER_MAX_PENDING_DEG` (360°) is a runaway guard to tune on the robot.
- **Proposed defaults:** the shaft-button role, the arm-hold mode, and the controller timeouts.
