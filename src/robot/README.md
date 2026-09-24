<!-- src/robot/README.md: Explain the Manual Mode teleoperation package (base driving, leader arm, KachiButton, Auto Catch alignment), how to run it, and what is verified. -->
# Robot Control

Robot integration for the Dino Egg Catch Challenge. The operating modes are specified in [docs/spec/operating-modes.md](../../docs/spec/operating-modes.md). Phase 1 is implemented here: **Manual Mode** (formerly Drive Mode), in which an attendee drives the LeKiwi base with the dino-controller and/or moves the dinosaur arm with the leader arm, a mode manager driven by KachiButton phrases, and a pygame signboard that shows the overhead, front, and wrist cameras, the mode, and the drive and arm status. Rerun is an optional operator view. Manual Mode's base driving is unchanged from the earlier implementation. Phase 2 step 1 adds front-camera egg detection and the first half of **Auto Catch**: `Hi!` checks that an egg is in view at a usable size and then drives the base until the egg sits at the best position in the front image (see "Auto Catch step 1: alignment").

**Stubs (no motion):** the catch after the alignment ("Aligned. Catch: not available yet"), Auto Release (`Thx` in Manual Mode: "Auto Release: not available yet"), and Full Self-Catching (FSC, mode "FSC (demo)") only change the signboard text. In FSC the base is sent zero velocities and the arm holds its pose.

Hardware adapters are kept separate from mode logic so behavior can be checked without moving a robot. Emergency stopping and communication-loss handling stay local (Mac application and Pi host), never behind a cloud service. Background and API references: [LeKiwi application development guide](../../docs/lekiwi-app-development.md) and [dino-controller protocol](../../docs/dino-controller-protocol.md).

## Modules

| File | Role | Hardware |
| --- | --- | --- |
| `config.py` | All tunables: Pi address, ZMQ ports, serial ports, leader arm port and id, arm engagement speed and tolerance, KachiButton phrases, camera index and Pi camera color order, loop rate, speed levels, input roles, signboard size and theme, egg detector HSV ranges, alignment target and controller, capture directory and key | None |
| `dino_controller_reader.py` | Serial JSON v0 lines -> immutable `ControllerState`; framing, validation, synchronization, sequence gaps; periodic `STATE` requests | Serial port (pyserial, imported lazily) |
| `controller_to_action.py` | `ControllerState` -> `{x.vel, y.vel, theta.vel}`; speed-level stepping | None (stdlib only) |
| `drive_state.py` | Frozen `DriveState` (speed level, Catch request, input lost, encoder rotation budget) and the stop/drive decision | None (stdlib only) |
| `kachi_phrases.py` | Typed text -> KachiButton commands (exact phrases, 1 s gap rule, bounded buffer) | None (stdlib only) |
| `mode_manager.py` | Frozen `AppState` (mode, action and its alignment state, voice input, notice and level, Stop latch), the command rules, Auto Catch start (size precondition) and finish | None (stdlib only) |
| `align.py` | Auto Catch base alignment: tapered speed toward the best position, EMA smoothing, rate limit, done / lost / timeout, trace rows | None (stdlib only) |
| `arm_follow.py` | Slow engagement toward the leader pose, then real-time following | None (stdlib only) |
| `manual_mode.py` | Per-frame composition: fold commands (`Hi!` with the egg size), step the alignment, decide base action and arm pose, arm status | None (stdlib only) |
| `display_status.py` | Frozen `DisplayStatus` sent to the signboard (mode, action, voice, notice and level, arm status, overlays) and the front-view overlays (target guide, best egg) | None (stdlib only) |
| `vision/egg_size.py` | Frozen `EggDetection` (normalized bbox, color, spots, area) and the size precondition `classify_size` | None (stdlib only) |
| `vision/egg_detector.py` | HSV color and shape detector: white body plus green/blue/red spots, pink basket excluded, convex/elliptical outline -> `detect_eggs`, `best_egg` | None (numpy, OpenCV loaded lazily) |
| `vision/frames.py` | Converts the RGB-ordered Pi front/wrist frames to BGR once, right after `observe()` | None (numpy) |
| `vision/align_trace.py` | Appends one CSV row per alignment frame to `captures/<stamp>-align.csv` for tuning | Disk (stdlib only) |
| `vision/timing.py` | Logs the detector's average time over the first 30 frames once | None (stdlib only) |
| `vision/capture.py` | Capture key: raw front/wrist/top PNGs plus a JSON record into `captures/` | Disk (OpenCV loaded lazily) |
| `vision/inspect.py` | Offline CLI `python -m robot.vision.inspect <image.png> [--rgb]`: print detections, write `<image>-detected.png` | None (OpenCV) |
| `leader_arm.py` | `SO100Leader` wrapper: `read_pose()` returns the six `arm_*` keys or `None` on a failed read | Leader arm serial port (LeRobot, imported lazily) |
| `top_camera.py` | `OpenCVCamera` wrapper for the overhead camera on the Mac (BGR; the Pi frames are converted to BGR by `vision/frames.py`) | USB camera |
| `lekiwi_adapter.py` | `LeKiwiClient` wrapper; captures the arm pose at connect, sends base keys with the commanded (or last) arm pose, zeros before disconnect | Network to the Pi |
| `signboard_layout.py` | Pure signboard layout (camera and status rectangles, letterboxing) and status text | None (stdlib only) |
| `signboard.py` | pygame signboard window: render cameras, overlays, and status, report ESC/close, typed text, and the capture key (child process only) | Display, keyboard |
| `signboard_protocol.py` | Pipe formats: packets (JSON header line with status and overlays + raw BGR frame bytes), close request, command lines | None (stdlib + numpy) |
| `signboard_process.py` | Signboard child entry point (`python -m robot.signboard_process`); matches phrases, prints commands (and `capture`); never imports LeRobot or cv2 | Display, keyboard |
| `signboard_client.py` | Parent side: starts the child, streams the newest packet from a writer thread, collects commands from a reader thread, reports liveness | Child process |
| `drive_loop.py` | One loop iteration and the 30 Hz loop: send, observe, BGR conversion, egg detection, capture; the display is any `DisplaySink` | All of the above |
| `teleop_drive.py` | Manual Mode entry point: CLI, connect in order, zeros-first shutdown | All of the above |

## Running Manual Mode

1. On the Raspberry Pi (`pi@lekiwi`, conda env `lerobot312`, inside `~/lerobot-dino-egg-catch-challenge`):

   ```bash
   python -m lerobot.robots.lekiwi.lekiwi_host --robot.id=dino_kiwi
   ```

2. On the Mac, from this repository root, use the conda env `lerobot312` (owner decision): `conda activate lerobot312`, or call `/opt/miniconda3/envs/lerobot312/bin/python` directly. The env has the LeRobot fork installed editable (`pip install -e ".[lekiwi,viz]"` in `../lerobot-dino-egg-catch-challenge`) plus pyserial and pygame-ce. Check that it imports the fork; the path must be under `lerobot-dino-egg-catch-challenge`:

   ```bash
   python -c "import lerobot; print(lerobot.__file__)"
   ```

   The root `pyproject.toml` describes the same dependencies for an alternative `uv` setup. The leader arm calibration `dino_leader_arm.json` must be present under `~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/` (see the guide, section 3); if it is missing or does not match the motors, LeRobot prompts on the terminal at connect (press ENTER to use the file or type `c` to recalibrate) and start-up waits for that answer. `--no-leader` skips the leader arm entirely (no connect, no prompt). Then start Manual Mode:

   ```bash
   PYTHONPATH=src python -m robot.teleop_drive
   PYTHONPATH=src python -m robot.teleop_drive --fullscreen --rerun
   PYTHONPATH=src python -m robot.teleop_drive --leader-port /dev/tty.usbmodemXXXX
   PYTHONPATH=src python -m robot.teleop_drive --no-leader --controller-port /dev/cu.usbserial-XXXX --no-camera
   ```

   Options: `--no-camera`, `--no-signboard`, `--fullscreen` (overrides `SIGNBOARD_FULLSCREEN`), `--rerun` (opt-in operator view), `--controller-port`, `--remote-ip`, `--leader-port` (default `LEADER_ARM_PORT`), `--no-leader` (the arm holds its pose; Manual Mode is drive-only). Defaults come from `config.py`. Find the overhead camera index with `lerobot-find-cameras opencv` and set `TOP_CAMERA_INDEX`. With `--no-signboard` there is no KachiButton input (and so no `Stop` phrase); only Ctrl+C stops the application.

   Devices are opened in this order: controller, robot (captures the arm pose to hold), leader arm, camera, signboard, Rerun. A leader arm that cannot be opened stops start-up; use `--no-leader` to run without it.

## KachiButton controls

The KachiButton types plain ASCII phrases into whichever window has keyboard focus. The signboard window keeps focus for the whole exhibit (owner decision; no global keyboard hook). The signboard child receives the text through pygame `TEXTINPUT` events, matches complete phrases (exact, case-sensitive, including spaces and punctuation; more than `KACHI_PHRASE_GAP_S` = 1 s between characters discards the partial phrase), and sends each command to the parent as one JSON line on its stdout: `{"v": 1, "type": "command", "command": "stop"}`.

| Phrase | Command | Manual Mode | FSC |
| --- | --- | --- | --- |
| `Go Go!` | `mode_toggle` | Switch to FSC (while stopped: release the Stop latch, stay in Manual Mode) | Switch to Manual Mode (while stopped: release the latch into Manual Mode) |
| `Hi!` | `hi` | Auto Catch step 1: size precondition, then base alignment (below) | Toggle voice listening (display only in Phase 1: "Listening..." / "Voice input ended", `mic on` in the status bar) |
| `Thx` | `thx` | Phase 1 stub: notice "Auto Release: not available yet", nothing moves | No action (owner decision) |
| `Stop` | `stop` | Emergency stop and reset (below) | Emergency stop and reset (below) |

Every mode switch first sends zero base velocities, clears pending encoder rotation, and disengages arm following. Presses other than `Stop` are ignored while an automatic action runs (the Auto Catch alignment). Notices stay for `NOTICE_SECONDS` (3 s).

**`Stop` is a latch** ([spec, section 2](../../docs/spec/operating-modes.md); owner decision, 2026-09-24). `Stop` switches to Manual Mode, cancels any action (including a running alignment), clears voice input, discards pending encoder rotation, disengages arm following, and latches the stopped state. While it holds, every frame sends zero base velocities (controller input is ignored and no rotation is queued), the arm holds its last commanded pose with torque kept on (the host only releases torque when it disconnects) and the leader is not read, `Hi!` and `Thx` are ignored, and another `Stop` keeps it stopped. The signboard shows `STOPPED` in the warning color with "Press Go Go! to resume" for as long as the latch holds. `Go Go!` releases it and always returns to Manual Mode (it does not toggle to FSC): one more zero-velocity frame, then the base follows the controller and the arm re-engages through the slow approach. Nothing moves again on its own after a stop. `Stop` does not close the application; ESC, the window close button, and Ctrl+C quit it through the zeros-first shutdown.

## Leader arm

The leader arm is an `so100_leader` on `LEADER_ARM_PORT` (`/dev/tty.usbmodem5A7A0179021`) with id `LEADER_ARM_ID` (`dino_leader_arm`), verified values from the guide. `get_action()` reports joints without the `arm_` prefix; `leader_arm.py` adds it and sends all six keys, including the gripper (the mouth).

- **Engagement (owner decision; numbers proposed):** when Manual Mode starts, after a mode switch, after `Stop`, and after a leader fault, the dinosaur arm moves toward the leader's pose at `ARM_ENGAGE_SPEED_DEG_S` (30 deg/s) per joint, starting from the last commanded pose. Real-time following begins once every joint is within `ARM_ENGAGE_TOLERANCE_DEG` (3°) of the leader. The gripper is in percent and uses the same numbers (30 %/s, 3 %). Both values are proposals to tune on the robot. The frame time used for the approach is clamped to 2 / `LOOP_HZ`, like the rotation budget.
- **Faults:** a failed read (bus error, disconnect, malformed reading) returns `None`; the arm holds its last commanded pose, following disengages so a recovered leader is approached slowly again, the base keeps working, and the signboard shows `LEADER ARM FAULT`. The adapter logs a warning once per failure streak and an info line on recovery.
- **Leader ignored:** in FSC, while the Stop latch holds, during the Auto Catch alignment, or with `--no-leader`, the arm holds its pose and the leader is not read.

## Auto Catch step 1: alignment

Owner decision (2026-09-24): eggs are white ellipsoids with green, blue, or red spots. In Manual Mode, `Hi!` looks at the newest **front camera** detection:

| Front camera at the press | Signboard (warning color, `NOTICE_SECONDS`) | Robot |
| --- | --- | --- |
| No egg detected | "No egg in view" | Nothing moves |
| Egg bbox height below `AUTO_CATCH_MIN_EGG_H` (0.15 of the frame) | "Egg too far" | Nothing moves |
| Egg bbox height above `AUTO_CATCH_MAX_EGG_H` (0.85) | "Egg too close" | Nothing moves |
| Otherwise | "Aligning...", mode line `MANUAL - AUTO CATCH` | Base aligns (below) |

There is no dialog to dismiss (attendees have no cursor): alerts are timed notices only. During the alignment the controller's base input and the leader arm are ignored (the arm holds its last pose, status `arm holding`), and `Hi!`, `Thx`, and `Go Go!` are ignored. The controller (`align.py`) drives the base toward the **best position**, the egg pose in the front image that the pick policy will start from: sideways from the horizontal error (egg right of the target -> move right, `-y`) and forward/backward from the bbox-height error (egg smaller than the target -> forward, `+x`), no rotation. The owner's first run on the robot (2026-09-24) moved too fast and oscillated, so the owner asked for a progressive slow-down. Each axis now commands `ALIGN_MAX_XY * clamp(error / full-speed error, -1, 1)`: full speed `ALIGN_MAX_XY` (0.06 m/s, below the 0.1 m/s slow level) at `ALIGN_FULL_SPEED_ERROR_CX` (0.18) or `ALIGN_FULL_SPEED_ERROR_H` (0.30), falling linearly toward zero near the target. An axis inside its tolerance gets 0, and a command below `ALIGN_MIN_XY` (0.015 m/s) becomes 0 so the base does not creep. The egg's cx and h are smoothed with an exponential moving average (`ALIGN_SMOOTHING` 0.5 = weight of the new sample; a missing detection leaves the average unchanged). The controller and the done test use the smoothed values. Each velocity changes by at most `ALIGN_MAX_ACCEL` (0.15 m/s²) × frame time per frame, which also ramps it down while the egg is briefly missing. Terminal results send zeros at once. `config.py` refuses values for which the command just outside a tolerance (`ALIGN_MAX_XY * tolerance / full-speed error`) would fall below `ALIGN_MIN_XY`, because the base would then stop short of "done" and wait for the timeout. With the current numbers that edge command is 0.0167 m/s (cx) and 0.016 m/s (h).

**If it still oscillates.** The Pi JPEG stream adds roughly 100-200 ms of latency, so the controller always reacts to a slightly old image. Lower `ALIGN_MAX_XY` or `ALIGN_MAX_ACCEL`, raise the full-speed errors, or lower `ALIGN_SMOOTHING` (heavier smoothing adds lag of its own). If that is not enough, the next step is move-stop-observe: drive for a short burst, stop, wait for a fresh frame, repeat. That mode is not implemented yet.

**Trace for tuning.** While aligning, the loop appends one row per frame (`t, cx_raw, h_raw, cx_smooth, h_smooth, x_vel, y_vel, result`) to `captures/<YYYYmmdd-HHMMSS>-align.csv` (stamp = alignment start; one line written per frame) and logs the path when the run ends. A failed write is logged once and tracing stops for that run; the alignment continues.

The run ends with:

- **done** after `ALIGN_DONE_FRAMES` (10) consecutive frames with the smoothed egg within `ALIGN_TOL_CX` (0.05) and `ALIGN_TOL_H` (0.08): "Aligned. Catch: not available yet" (the catch is Phase 3);
- **lost** after `ALIGN_LOST_FRAMES` (15) consecutive frames without an egg (the command ramps toward zero meanwhile): warning "Egg lost";
- **timeout** after `ALIGN_TIMEOUT_S` (15 s): warning "Could not align".

Each result sends zero velocities for that frame and returns to Manual Mode with arm following disengaged, so the arm re-syncs to the leader slowly.

**Detection.** The detector (`vision/egg_detector.py`) runs every frame in Manual Mode on the front frame, in the parent process: HSV -> (white-body mask `EGG_BODY_HSV` OR the spot masks `EGG_SPOT_HSV`) AND NOT the pink basket (`BASKET_HSV`) -> morphological open (`EGG_OPEN_KERNEL_PX`, drops thin clutter such as cables and tarp sparkles) and close (`EGG_CLOSE_KERNEL_PX`) -> outer contours with at least `EGG_MIN_AREA_PX` and a bbox aspect inside `EGG_ASPECT_RANGE` -> shape test: solidity (contour area / convex hull area) at least `EGG_MIN_SOLIDITY` (0.85), and, for contours at least `EGG_BORDER_MARGIN_PX` away from every frame edge, contour area / fitted-ellipse area inside `EGG_ELLIPSE_FILL_RANGE` (0.75-1.25). A partly visible egg cut by the border is a truncated ellipse, still convex, so it keeps only the solidity test and is reported with `touches_border`. Spot blobs of at least `EGG_MIN_SPOT_AREA_PX` are then counted per color inside the filled contour (`EGG_MIN_SPOTS` needed); the color with the most spot pixels names the egg, and the largest egg is used. The basket is removed from every mask, so it never joins an egg or adds spots. This fixes the robot bug of 2026-09-24, where the basket behind an egg became one frame-wide "red egg". Red spots need S >= 150 to stay apart from the pink. Spots stay in the component mask because the eggs' large, dark spots would otherwise split the white body. `BASKET_HSV` will be reused by the Auto Release basket detector. The signboard's front view always shows the target as an egg-shaped outline labelled "place the egg here" just below it (also a guide for attendees driving by hand), and the best egg as a green outline ("green egg", label above it) when its size is usable or an accent-colored outline ("green egg: too far") otherwise. The detector's average time over the first 30 frames is logged once at INFO.

**Pi camera colors.** `LeKiwiClient` returns the front and wrist frames RGB-ordered (see `PI_CAMERA_COLOR_ORDER` in `config.py` for the verified chain in the fork); the loop converts them to BGR once, right after `observe()`, so the detector, the signboard, captures, and Rerun all get BGR like the overhead camera.

**Values are placeholders to calibrate at the venue.** The best position (`ALIGN_TARGET_CX` 0.49, `ALIGN_TARGET_CY` 0.53, `ALIGN_TARGET_W` 0.62, `ALIGN_TARGET_H` 0.59) is the detector's own measurement of capture `20260924-220742`, taken by the owner with the egg at the best position, so target and detector agree. The HSV ranges were checked against the captures `20260924-220336` and `20260924-220404`. The tolerances and speeds are proposals. Recalibrate at the venue:

1. **Capture the reference frame.** Place an egg at the pose the pick policy starts from and press `c` in the signboard window (a KEYDOWN, not a KachiButton phrase). The raw front, wrist, and top frames are saved as `captures/<YYYYmmdd-HHMMSS>-{front,wrist,top}.png` with `<stamp>.json` (detections with normalized bbox, color, and size class, plus the mode); the notice says "Captured". `captures/` is git-ignored.
2. **Tune offline.** `PYTHONPATH=src python -m robot.vision.inspect captures/<stamp>-front.png` prints every detection (cx, cy, w, h, color, spots, area, solidity, border contact, size class) and writes `<stamp>-front-detected.png` with the boxes and the current target drawn. Adjust `EGG_BODY_HSV` and `EGG_SPOT_HSV` (OpenCV ranges: H 0-179, S and V 0-255) until each egg is one box that covers its shadowed lower half, then copy the reference detection's cx, cy, w, h into `ALIGN_TARGET_*`. `--rgb` reads an RGB-ordered image, such as a signboard screenshot of the Pi cameras taken before the color fix.

## Display

The attendee display is a pygame window (owner decision, 2026-09-24): the overhead camera large on the left, the front and wrist cameras stacked on the right (the front view with the Auto Catch overlays), and a three-line status bar. Line 1 (large) is the mode: `MANUAL` or `FSC (demo)`, followed by the running action when there is one, or `STOPPED` while the Stop latch holds. Line 2 is "Press Go Go! to resume" while stopped; otherwise the current notice if present; otherwise, in Manual Mode, the drive status (`DRIVE`, `INPUT LOST`, and `CATCH!` only if the Catch role is re-enabled), and in FSC the voice hint "Press Hi! to talk" or "Listening... (Hi! to end)" (the controller is not used in FSC, so its status is not shown there). Line 3 is the arm status (`arm holding`, `arm syncing`, `arm following`, `LEADER ARM FAULT`, `no leader arm`), `mic on` while FSC voice input is toggled on, the speed level as footprints (fixed by config), and the held directions plus a rotation symbol (↺/↻, ASCII `CCW`/`CW`) while an encoder rotation is pending. The bar turns the warning color while stopped, while a warning notice is shown (a rejected Auto Catch, "Egg lost", "Could not align", "Capture failed"), and, in Manual Mode, on input loss. A missing frame shows "no signal". Unicode symbols are used only when pygame's default font has them; otherwise ASCII fallbacks (`#`/`-`, `UP`/`RIGHT`) are shown.

The signboard runs as a separate interpreter (`python -m robot.signboard_process`, started by `SignboardClient`) because opencv (pulled in by LeRobot) and pygame each bundle their own `libSDL2` on macOS, and both copies cannot safely live in one process. The control loop sends it one packet per loop frame over the child's stdin; only the newest packet is kept, so the loop never blocks on the display. KachiButton commands come back on the child's stdout (nothing else is written there; logs go to stderr) and are collected by a reader thread into a queue that the loop drains each frame. The parent process never imports pygame and the signboard process never imports LeRobot or cv2; tests assert both.

The window has no buttons, and ordinary keys do not control the robot: only complete KachiButton phrases do. The one extra key is `c` (`CAPTURE_KEY`), which only saves a capture; no KachiButton phrase contains it. ESC or the window close button (or the signboard process exiting for any reason) ends the application through the normal shutdown, so zero velocities are sent before disconnecting. The colors in `DEFAULT_THEME` are a placeholder Jurassic palette; artwork (wood-sign frames, fonts, footprint icons) will come later as image assets.

## Input mapping

| Controller input | Manual Mode action | Status |
| --- | --- | --- |
| Joystick up / down / left / right | Translate forward / backward / left / right (`x.vel`, `y.vel`; `LEFT_RIGHT_ROLE = "strafe"`) | Owner decision, 2026-09-24 |
| Rotary encoder | Rotate right (cw) / left (ccw); one click turns the base by half the knob's own angle per click (360° / detents × 0.5; currently assumed 20 detents = 9°), open-loop; the detent count needs the counted test. Queued rotation is capped at `ENCODER_MAX_PENDING_DEG` (360°, one knob turn) | Owner decision, 2026-09-24 |
| (none) | Speed level fixed at `INITIAL_SPEED_INDEX` (slow: 0.1 m/s, 30 deg/s); no input changes it | Owner decision, 2026-09-24 |
| Shaft button | Reserved: no role (`SHAFT_BUTTON_ROLE = "none"`). It stays wired and reported for a later use. Catch moved to `Hi!` and stop is the second KachiButton. The former Catch role (`"catch"`: the base stops while held) remains available in `config.py` and is covered by tests | Owner decision, 2026-09-24 |

Encoder clicks add a rotation budget in degrees; each frame the loop sends `theta.vel = ±theta` and subtracts what that speed covers in the measured frame time (clamped to 2 / `LOOP_HZ`). `ENCODER_DEGREES_PER_STEP` is derived as `360 / ENCODER_CLICKS_PER_REVOLUTION * ENCODER_ROTATION_SCALE` (scale 0.5, owner decision 2026-09-24). `ENCODER_CLICKS_PER_REVOLUTION = 20` is a placeholder until the detent count is measured (see "Detent calibration", still pending in [dino-controller-validation.md](../../docs/dino-controller-validation.md)). This is commanded, not measured, rotation, so the real heading change also depends on the wheels and floor. Rotating while translating is allowed. Contradictory directions cancel. Every command also carries six arm positions, because the fork's host fails on actions without arm keys (code reading, not tested on the robot): the pose captured at connect until the leader arm is engaged, then the commanded leader pose, and the last commanded pose whenever the arm is held. Arm values are never derived from controller input.

## Safety behavior

- **Alignment is slow, cancellable, and bounded.** The Auto Catch alignment never exceeds `ALIGN_MAX_XY` (0.06 m/s, below the slow level) on either axis, changes speed by at most `ALIGN_MAX_ACCEL` (0.15 m/s²), and never rotates; `Stop` cancels it at once, and it ends by itself after `ALIGN_TIMEOUT_S` (15 s) or when the egg is lost. Loss of dino-controller input does not stop it (the attendee is not driving), but `Stop`, ESC/window close, Ctrl+C, and the host watchdog still apply.
- **Stops clear pending rotation.** Input loss, `Stop` and its latch, every mode switch, FSC, and the optional Catch role all reset the encoder rotation budget to zero, so rotation never resumes on its own after a stop.
- **Input lost -> zeros.** Until a valid `state` snapshot is received, after `ready`, `error`, or a sequence gap, and whenever no valid controller message arrives for `CONTROLLER_INPUT_TIMEOUT_S` (0.5 s), the loop sends zero velocities. The reader requests `STATE` every `CONTROLLER_STATE_POLL_INTERVAL_S` (0.2 s) because the firmware is silent while inputs are unchanged.
- **`Stop` latches zeros and a held arm until `Go Go!`; mode switches send zeros for one frame.** See "KachiButton controls". This is a software stop through the signboard process and the loop (tens of milliseconds); the host watchdog remains the last line.
- **No jump on engagement.** The arm approaches the leader at a limited rate before following it; a leader fault holds the arm and forces a new slow approach.
- **Manual Mode needs the arm pose.** The application does not start unless the arm pose can be read at connect; every command, including stop commands, carries a full arm pose.
- **Ctrl+C, ESC/window close, or any loop error -> zeros, then disconnect.** Shutdown sends zero velocities before closing the robot connection, then closes the leader arm, camera, controller, and signboard.
- **Host watchdog is the backstop.** The Pi host stops the base if no command arrives for 500 ms; the application does not rely on it.

## Tests

Hardware-free unit tests (no serial port, camera, or robot is opened):

```bash
PYTHONPATH=src /opt/miniconda3/envs/lerobot312/bin/python -m unittest discover -s tests/robot -v
```

Do not add `-t .`: the `lerobot312` environment has an unrelated `tests` package in `site-packages` that shadows this repository's `tests/` directory, so `tests.robot` cannot be imported. The invocation above matches the `tests/tactile_fingertip` suite.

The OpenCV tests (detector on synthetic frames, capture, inspect CLI) live in `tests/robot/vision/` without an `__init__.py` and are run by `test_vision_suite.py` in a separate interpreter, because OpenCV and pygame both bundle `libSDL2` and must not load into one process. The control-loop tests patch the detector out, so the main test process never loads OpenCV. To run the OpenCV tests on their own: `PYTHONPATH=src /opt/miniconda3/envs/lerobot312/bin/python -m unittest discover -s tests/robot/vision -v`.

## Verified vs. proposed

- **Verified without hardware:** the unit tests above pass. Base driving: controller framing, parsing, synchronization, action mapping, the encoder rotation budget, input-lost stops and the optional Catch role, arm-pose capture and nine-key action content with a fake client. Phase 1 additions (unit tests only): KachiButton phrase matching, the mode manager rules including the Stop latch, the FSC voice hints, slow engagement and following, the leader-arm adapter with a fake teleoperator, the adapter updating its held pose, the per-frame composition and loop wiring with fake devices (Stop, mode switch, leader following, leader fault), the protocol status fields and command lines, headless pygame rendering and `TEXTINPUT` collection with `SDL_VIDEODRIVER=dummy`, a fake child sending command lines to `SignboardClient`, a headless round trip through a real signboard child, and the import isolation of both processes. The hardware modules import and `--help` runs in the `lerobot312` environment.
- **Phase 2 step 1 (unit tests; the owner's first robot run on 2026-09-24 found the basket false detection and the oscillation, both addressed here but not yet re-run on the robot):** the size precondition and its alerts, the alignment controller (signs, taper, tolerance zero, deadband, smoothing, rate limit, done / lost / timeout, trace rows), the loop wiring with a patched detector (controller and leader ignored, input loss not stopping it, Stop cancelling it, capture), the RGB -> BGR conversion, the overlays through the protocol and headless rendering, the capture key, and the detector on synthetic frames. Offline checks on real front captures from the robot (not unit-test fixtures): `captures/20260924-220336-front.png` (egg with the pink basket behind it) and `20260924-220404-front.png` (same egg, no basket) each give one green egg at cx 0.39, cy 0.47, w 0.45, h 0.43, and nothing on the basket or the white pipe. The earlier channel-swapped screenshot (read with `--rgb`) gives one green egg at cx 0.561, cy 0.556, w 0.659, h 0.637. Capture `20260924-220742-front.png` (the owner's best-position reference) gives cx 0.485, cy 0.526, w 0.617, h 0.594, the source of `ALIGN_TARGET_*`. The shadowed bottom of an egg stays outside its box. Thresholds still need venue calibration.
- **Owner-tested on the robot (2026-09-24):** Manual Mode end to end: the owner drove the base with the dino-controller, puppeteered the arm with the leader arm and caught an egg, toggled Manual Mode and FSC with `Go Go!` on a KachiButton in the real signboard window, and confirmed that after moving the arm in FSC the follower re-synchronized slowly on return to Manual Mode. Not yet reported: `Stop` on the second KachiButton, the measured rotation per encoder click, and the overhead camera view quality. Each further physical run still needs the scope, reachable stop control, operating area, and verified limits agreed with the operator (see `AGENTS.md`).
- **Owner decisions (2026-09-24):** Manual Mode, the KachiButton mapping and signboard focus, `Stop` as a full stop with the arm held, the shaft button without a role, slow engagement, joystick translation, encoder rotation, fixed speed level. One click turns the base by half the knob's own angle per click (`ENCODER_ROTATION_SCALE = 0.5`); `ENCODER_CLICKS_PER_REVOLUTION` (20) is an unmeasured placeholder, and `ENCODER_MAX_PENDING_DEG` (360°) is a runaway guard to tune on the robot.
- **Proposed defaults:** `ARM_ENGAGE_SPEED_DEG_S` (30) and `ARM_ENGAGE_TOLERANCE_DEG` (3), `NOTICE_SECONDS` (3), the controller timeouts, and every Auto Catch value in `config.py` (HSV ranges, detector limits, best position, tolerances, size window, gains, speed limits, frame counts, timeout).
