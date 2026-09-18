<!-- docs/dino-controller-validation.md: Define wiring checks, reproducible Arduino builds, and acceptance evidence without claiming physical tests. -->
# Dino Controller Build and Validation Plan

Draft — 2026-09-19. Companion to the [detailed specification](dino-controller.md). Stage 1 LED blinking was visually accepted. Stage 2 passed automated software tests, and the owner confirmed correct basic operation after GPIO remapping on 2026-09-19. Extended physical checks and Stage 3 encoder checks remain pending unless explicitly recorded. Current build/upload evidence is in the [sketch README](../src/dino-controller/README.md).

## Stage 1. Red LED smoke test

The Stage 1 sketch at commit `120c321` blinked red for one second and off for one second. That visual check is complete; the current Stage 2 sketch uses direction colors instead. See the [sketch README](../src/dino-controller/README.md) for instructions and recorded results.

Validation record (2026-09-19): Arduino compilation and upload passed, and the project owner confirmed correct red blinking on the connected board. Stage 1 is accepted. Later joystick validation is recorded below; encoder validation remains pending.

## Stage 2. Identify joystick wiring before power-on

1. Disconnect USB power and disconnect the joystick harness from the ESP32. Use a multimeter in continuity mode on the unpowered joystick alone.
2. Photograph the connector, latch, viewing side, and installed player-facing orientation. Assign temporary labels P1–P5 in the photograph; do not borrow numbering from the HAYABUSA image.
3. With the stick neutral, verify its normally-open directional contacts do not conduct.
4. Move UP from the player's viewpoint. Find the pair of harness wires that becomes continuous. Repeat for DOWN, LEFT, and RIGHT.
5. Identify the one wire shared by all four directional pairs: the candidate common/GND. Each other wire should close only for its respective direction. If no consistent common exists, stop assuming a passive common-ground joystick and inspect the actual switch terminals.
6. Test diagonals if supported by the restrictor: common should connect to two adjacent directions together. Confirm release returns to open contacts. Mechanical switch placement under the stick can be opposite to the direction of handle movement.
7. Record results, then connect the verified common and four contacts to the proposed GPIOs or revise the configuration to match existing wiring. Keep all five joystick wires away from the supply rails except the verified common-to-GND connection.

| Player action | Conducting pair | Verified GPIO / result |
| --- | --- | --- |
| Neutral | None expected | Pending |
| UP | Common ___ / wire ___ | GPIO26, observed by owner; continuity record pending |
| DOWN | Common ___ / wire ___ | GPIO27, observed by owner; continuity record pending |
| LEFT | Common ___ / wire ___ | GPIO21, observed by owner; continuity record pending |
| RIGHT | Common ___ / wire ___ | GPIO25, observed by owner; continuity record pending |
| UP+RIGHT | Common / UP / RIGHT | Pending; verify restrictor permits it |

The reference order GND / DOWN / UP / RIGHT / LEFT is a hypothesis only. Store the measured connector orientation and mapping with the eventual test record.

Direction calibration (owner report, 2026-09-19): the initial firmware reported `down` for physical UP, `left` for physical DOWN, `right` for physical LEFT, and `up` for physical RIGHT. These observations establish the GPIO assignments above. Keep the wires in place and remap the logical directions in firmware. After the corrected firmware was uploaded, the owner reported that it worked correctly on 2026-09-19. This accepts the Stage 2 basic operation check; individual repetition counts, diagonal combinations, and latency measurements were not reported.

### Stage 2 Serial Monitor and RGB check

After the unpowered wiring checks, upload the Stage 2 sketch and open Serial Monitor at **115200 baud, 8N1**, with newline enabled. If startup output was missed, send `STATE`. On 2026-09-19, upload succeeded on `/dev/cu.usbserial-110`, and a bounded serial check received a valid all-false joystick snapshot in response to `STATE`. The checker then closed and released the port. This confirms the serial response, but not physical wiring, directions, or LED colors. Check these actions from the player's viewpoint:

| Action | Serial state | RGB LED | Physical status |
| --- | --- | --- | --- |
| Neutral / release | All four fields false | Off | Pending |
| UP | Only `up` true | Red | Included in owner acceptance of corrected basic operation |
| DOWN | Only `down` true | Green | Included in owner acceptance of corrected basic operation |
| RIGHT | Only `right` true | Blue | Included in owner acceptance of corrected basic operation |
| LEFT | Only `left` true | White | Included in owner acceptance of corrected basic operation |
| UP+RIGHT or UP+LEFT | Both relevant fields true | Red | Pending |
| DOWN+RIGHT or DOWN+LEFT | Both relevant fields true | Green | Pending |
| Hold one direction | No repeat records | Corresponding color stays on | Pending |

A wrong direction/color requires checking the wire-to-GPIO mapping, not assuming the reference connector image is correct. Record the observed results before adding the encoder. The LED confirms the same debounced state sent over USB; it does not independently prove the common wire is correct.

Automated check command: `python3 tests/dino_controller/run_tests.py`. The tests compile the actual sketch against fake GPIO, time, RGB, and UART interfaces with AddressSanitizer and UndefinedBehaviorSanitizer. They cover switch bounce, independent debounce, held startup, release, diagonals, opposing contacts, neutral/color priority, timer wraparound, `STATE` framing, bounded commands, partial writes, and overflow recovery. Hardware GPIO voltages and actual switch directions remain separate physical checks.

## Stage 3. Verify the rotary encoder

1. With power disconnected, verify terminal labels and check SW-to-GND continuity when pressed. Inspect whether onboard resistors pull signal pins toward `+`.
2. Power only the encoder supply from 3V3/GND initially; keep its signal wires disconnected from ESP32 inputs while checking that CLK/DT/SW voltages stay within the ESP32 input range.
3. Connect CLK→GPIO32, DT→GPIO33, and SW→GPIO23 after that check. Confirm released/pressed SW polarity.
4. Use the planned diagnostic build or a logic analyzer to observe slow clockwise and counterclockwise motion. Count physical detents per revolution, electrical transitions per detent, and A/B rest states.
5. Choose full-step or half-step decoding and the rest phase from measurements. Flip the sign configuration if physical CW reports CCW; do not compensate in the PC application.
6. Save a short trace for each direction and for bounce/reversal. Include these measured patterns in later decoder tests.

For a synthetic example only, A=CLK and B=DT may traverse `11 → 10 → 00 → 01 → 11`; reverse traversal must yield the opposite sign. Which traversal is physical CW remains unverified.

## 3. Arduino build target

| Setting | Initial proposal |
| --- | --- |
| Framework | Arduino-ESP32, Espressif Systems |
| Board | ESP32 Dev Module |
| FQBN | `esp32:esp32:esp32` |
| Flash / PSRAM | 4 MB / Disabled, subject to actual module confirmation |
| CPU | Board default, initially 240 MHz |
| Partition | Default 4 MB scheme |
| Core Debug Level | None; keep diagnostic output separate from protocol frames |
| Additional libraries | None |

Observed development environment on 2026-09-19: Arduino IDE **2.3.10**, its bundled Arduino CLI **1.5.1**, and installed ESP32 core **3.3.11**. The local `boards.txt` defines `esp32.name=ESP32 Dev Module`. CLI `version` and `compile --help` were checked. These observations do not establish physical input validation.

The CLI was not on the shell PATH, but the Arduino IDE bundled executable was available. Initial builds should use and record the installed 3.3.11 baseline; change versions only with a recorded reason and rerun acceptance tests.

### Arduino IDE procedure after implementation

1. Open `src/dino-controller/dino-controller.ino`.
2. In Boards Manager, select the installed **esp32 by Espressif Systems 3.3.11** package.
3. Select **ESP32 Dev Module** and the settings above, adjusting flash/PSRAM only after identifying the real module.
4. Run **Verify** to compile. Record the complete build result and flash/RAM usage.
5. For a physical test, choose the verified device port and run **Upload**. In Stage 1, visually check the red/off blink; the sketch emits no serial messages.
6. In Stages 2 and 3, open Serial Monitor at **115200 baud** with newline for `STATE`. Stage 2 uses experimental joystick-only reports; Stage 3 uses the complete v1 protocol. Close Serial Monitor before another application opens the same port.

### CLI equivalent after implementation

Run from the repository root. These commands compile the current Stage 2 joystick diagnostic sketch. They do not upload or establish joystick/encoder operation.

```sh
ARDUINO_CLI='/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli'
"$ARDUINO_CLI" core list
"$ARDUINO_CLI" board details --fqbn esp32:esp32:esp32
"$ARDUINO_CLI" compile \
  --fqbn esp32:esp32:esp32 \
  --board-options FlashSize=4M,PSRAM=disabled,PartitionScheme=default,DebugLevel=none \
  --warnings all \
  --build-path "$PWD/build/dino-controller" \
  src/dino-controller
```

On a clean machine, install Arduino CLI first, then install the pinned core using the official package index:

```sh
arduino-cli core update-index \
  --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core install esp32:esp32@3.3.11 \
  --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
```

Do not infer a serial port from a copied example. Enumerate ports with the board connected and verify the device. The Stage 1 connection and upload results are recorded in the sketch README. No joystick or encoder physical tests are implied by them.

## 4. Hardware-independent acceptance coverage

Tests must exercise the production logic with synthetic times and transitions, without requiring a board. Keep them outside the Arduino sketch directory.

| Test | Expected result |
| --- | --- |
| Stable press/release and contact bounce | One event after each accepted 10 ms stable interval; no duplicates |
| Two joystick inputs changing in one scan | One full-state message containing both changes |
| Diagonal contacts closing at different times | Correct intermediate and final states; both contacts can remain active |
| Held and opposing inputs | No auto-repeat; all physical accepted bits retained |
| Complete forward/reverse quadrature cycles | One event per calibrated detent, correct opposite signs |
| CW then CCW before the main loop drains | Two ordered events, never a net-zero disappearance |
| Partial reversal, bounce, invalid jump | No phantom completed step; resynchronization after invalid input |
| Startup at an arbitrary phase | No rotation until a subsequent qualified detent completes |
| Full-step and half-step calibration | Correct counts for each measured rest-state pattern |
| Queue overflow and partial TX write | Complete JSON frames, explicit error, then fresh state; no silent loss |
| Snapshot while input changes | Snapshot/event ordering never restores an older held state |
| Protocol framing and command bounds | Valid JSON, typed payloads, `STATE` LF/CRLF, bounded invalid input |
| Counter/timer rollover | Correct debounce across `millis()` wrap; sequence wraps modulo 2^32 |

Debounce, color feedback, and diagnostic transport tests are implemented in Stage 2. Quadrature tests will be added in Stage 3. The Stage 1 blink was compiled, uploaded, and visually accepted. Do not substitute tests that merely check whether source strings exist.

## 5. Physical acceptance tests

Operate only the input controller and serial receiver for this milestone; robot movement is not part of these tests.

| Test | Proposed pass criterion | Status |
| --- | --- | --- |
| Power-on neutral | `ready` then all-false `state`; no phantom input | Not run |
| Power-on held input | Snapshot accurately reports held contacts; no fabricated rotation | Not run |
| Four joystick directions | 20 press/release cycles per direction with correct states and no duplicates | Not run |
| Supported diagonals | 10 cycles per supported diagonal, both contacts visible and released correctly | Not run |
| Encoder switch | 20 press/release cycles; one event per accepted transition | Not run |
| Encoder slow rotation | 20 detents CW and 20 CCW; exactly 20 matching events each | Not run |
| Encoder fast rotation | Controlled 20 detents/second for 5 seconds each direction: 100 matching events, no loss | Not run |
| Reversal and concurrent input | Ordered CW/CCW events retained while joystick and SW are operated | Not run |
| Idle | No input events for 60 seconds after stabilization, excluding requested state replies | Not run |
| Reconnect without board reset | `STATE` recovers a held direction even when the press happened before the port opened | Not run |
| Reset / port auto-reset | Boot text tolerated; new session obtains a current state | Not run |
| Disconnect | PC-side receiver clears held inputs and requires synchronization on reopen | Not run |
| Queue fault injection | Error reported, no malformed frames, recovery snapshot restores current levels | Not run |
| Input latency | Proposed target: ≤30 ms from stable physical transition/detent completion to a complete PC frame under normal load | Not run |

Measure rate/count and latency with a repeatable fixture or signal capture when making a quantitative claim; hand spinning alone is exploratory. `ms` reports device time, so compare against a shared physical trace or synchronized measurement rather than subtracting unrelated PC and ESP32 clocks.

## 6. Evidence to record

Record firmware revision, IDE/CLI/core versions, FQBN and board options, GPIO mapping, encoder calibration, cable length, configuration values, build log, test results, connector photos, and representative serial captures. Separate simulated tests, compilation, successful upload, and measured physical results. An untested item remains pending.
