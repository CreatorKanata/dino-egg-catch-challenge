<!-- src/dino-controller/README.md: Explain Stage 2 joystick wiring, direction colors and USB diagnostics. -->
# Dino Controller

The current Arduino sketch checks the **four-switch arcade joystick** using RGB direction feedback and USB serial reports. It debounces contacts, reports presses and releases, and preserves both directions on diagonals. Rotary encoder inputs are reserved for Stage 3 and are not read yet.

## Wiring

Disconnect USB before wiring. Identify the joystick common wire by continuity first; the supplied HAYABUSA connector picture is a reference for another product, not proof of this joystick's wire order.

| Joystick contact | ESP32 GPIO / terminal | LED when active alone |
| --- | --- | --- |
| Common | GND | — |
| UP | 26 | Red |
| DOWN | 27 | Green |
| RIGHT | 25 | Blue |
| LEFT | 21 | White |
| Neutral | No contacts active | Off |

The direction-to-GPIO mapping follows the owner's handle-motion observations on 2026-09-19. Keep the existing wires in place; `config.h` corrects the logical names. The previous mapping reported DOWN for physical UP, LEFT for DOWN, RIGHT for LEFT, and UP for RIGHT.

The intended passive joystick has no VCC connection. Each contact uses `INPUT_PULLUP`; closing it to common GND reads LOW and becomes `true`. Directions are from the player's viewpoint. See the [wiring checks](../../docs/dino-controller-validation.md) and [Freenove pinout](../../images/dino-controller/Freenove-ESP32-Dev-Board.png).

The onboard WS2812 is on **GPIO16**, separate from the GPIO2 LED. Brightness is 32/255 per active channel. **Up/down take priority on diagonals:** UP+LEFT/RIGHT shows red; DOWN+LEFT/RIGHT shows green. Serial still reports both active directions. For contradictory contacts, deterministic LED priority is UP, DOWN, RIGHT, LEFT; serial preserves all bits for diagnosis.

The LED changes only after the same 10 ms debounce used by serial output. This replaces the previous blinking pattern; neutral now remains dark. GPIO scanning runs on a 1 ms schedule without `delay` calls.

## Check in Arduino Serial Monitor

1. Open `dino-controller.ino`, select **ESP32 Dev Module** (`esp32:esp32:esp32`) with the **esp32 by Espressif Systems 3.3.11** core, 4 MB flash and PSRAM Disabled.
2. Verify/upload, then select the connected USB serial port and open Serial Monitor at **115200 baud**.
3. At startup, a `ready` record shows the pin map, followed by the initial joystick state. A held contact is included in that initial state.
4. Move each direction, release it, and try supported diagonals. The corresponding fields become `true`; release restores `false`. Check the LED colors against the table above.
5. Holding the stick produces no repeated records. If output was missed, send **`STATE`** with newline (LF or CRLF) to request the current state.

Illustrative UP, then neutral reports:

```json
{"v":0,"type":"joystick","seq":2,"ms":500,"up":true,"down":false,"left":false,"right":false}
{"v":0,"type":"joystick","seq":3,"ms":900,"up":false,"down":false,"left":false,"right":false}
```

This is the experimental Stage 2 protocol (`v: 0`), not the future combined controller v1. A fixed 64-event queue allows input scanning while serial data drains. If it overflows, an `event_overflow` error is followed by a fresh joystick snapshot. Ignore ROM boot text before the JSON messages.

If directions are swapped, disconnect USB and correct the verified wire mapping or the pin constants in `config.h`. If everything remains false, check common GND and each contact's continuity. Avoid inferring correct wiring merely from a neutral all-false report: disconnected inputs also read inactive.

## Build and software tests

Run from the repository root:

```sh
python3 tests/dino_controller/run_tests.py
ARDUINO_CLI='/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli'
"$ARDUINO_CLI" compile \
  --fqbn esp32:esp32:esp32 \
  --board-options FlashSize=4M,PSRAM=disabled,PartitionScheme=default,DebugLevel=none \
  --warnings all --build-path "$PWD/build/dino-controller" \
  src/dino-controller
```

The host tests need Python 3 and a C++17 compiler with AddressSanitizer/UndefinedBehaviorSanitizer support (Clang on this development Mac). They require no extra Arduino or Python packages. They execute the real sketch with simulated GPIO, time, RGB, and partial UART writes, then validate the emitted JSON.

## Validation record

- **Stage 1:** commit `120c321` contains the red 1-second on/off smoke test; the owner confirmed visible blinking on 2026-09-19.
- **Stage 2 software, 2026-09-19:** four host test cases passed, including native debounce, command/queue, color-priority, and sketch integration assertions under sanitizers. Arduino CLI 1.5.1 / ESP32 core 3.3.11 compilation passed: 289,325 / 1,310,720 bytes flash (22%); 23,940 / 327,680 bytes static RAM (7%).
- **Stage 2 hardware, 2026-09-19:** upload to `/dev/cu.usbserial-110` passed with flash hashes verified and reset completed. A bounded 115200 baud, 8N1 serial check sent `STATE` and received a valid v0 joystick snapshot with all four inputs false (`seq: 2`, `ms: 148432`); the check closed and released the port immediately afterward. That serial-only check did not establish physical direction or color behavior; the later owner confirmation is recorded below.
- **Direction calibration, 2026-09-19:** the owner identified UP on GPIO26, DOWN on GPIO27, LEFT on GPIO21, and RIGHT on GPIO25. Firmware and regression tests now use these observations. All four host test cases and the new physical-GPIO mapping assertions passed; Arduino compilation and upload passed again with the same flash/RAM usage. A bounded `STATE` check received an all-false v0 snapshot (`seq: 2`, `ms: 17961`) and released the port. The owner subsequently confirmed that the corrected joystick worked correctly on 2026-09-19. Stage 2 basic operation is accepted; detailed cycle-count, diagonal, and latency measurements were not reported.
- **Stage 3:** rotary encoder implementation and calibration are pending.

## Detailed specifications

- [Controller stages, GPIO allocation, debounce, and planned v1 protocol](../../docs/dino-controller.md)
- [Continuity checks and physical acceptance tests](../../docs/dino-controller-validation.md)

References: [Freenove setup](https://docs.freenove.com/projects/fnk0090/en/latest/fnk0090/codes/C/Preface.html), [Espressif GPIO API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/gpio.html), [Espressif Serial API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/serial.html).
