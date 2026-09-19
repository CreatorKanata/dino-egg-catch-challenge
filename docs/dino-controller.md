<!-- docs/dino-controller.md: Describe the implemented controller and link wiring, protocol, and development guides. -->
# Dino Controller Specification

As implemented on **2026-09-19**. The owner accepted red LED blinking, corrected joystick directions, and rotary encoder operation after CW/CCW inversion. The current firmware remains **protocol v0**; acceptance does not change the version in `config.h`. Quantitative count, speed, and latency checks remain separate from basic acceptance.

## Documentation map

| Guide | Contents |
| --- | --- |
| [Illustrated wiring](dino-controller-wiring.md) | Board pin positions, component images, verified GPIOs, connector limitations |
| [Development and debugging](dino-controller-development.md) | Arduino IDE/CLI setup, build, upload, monitoring, troubleshooting |
| [JSON protocol v0](dino-controller-protocol.md) | Every message, field, command, startup sequence, and recovery behavior |
| [Encoder calibration](dino-controller-encoder.md) | Direction, electrical steps, push switch, acquisition |
| [Validation record](dino-controller-validation.md) | Completed stages, software evidence, remaining measurements |
| [Sketch entry point](../src/dino-controller/README.md) | Firmware location and quick start |

![Controller wiring with verified GPIO labels](../images/dino-controller/controller-wiring.png)

The drawing directly embeds the board cropped from the supplied Freenove pinout, preserving its original pixels and pin layout with all annotations outside the board. The HAYABUSA inset is a reference for another joystick: its connector order is not verified for this unit. The functional contact-to-GPIO mapping follows the owner's actual handle movements. See the [wiring guide](dino-controller-wiring.md) before identifying an unfamiliar harness.

## Hardware and scope

The controller uses a Freenove ESP32-WROOM-32E development board, a passive four-contact arcade joystick, and a rotary encoder with `CLK`, `DT`, `SW`, `+`, and `GND` terminals. The board USB connector supplies power and carries UART0 through the onboard USB–UART bridge to the PC. No extra TX/RX wiring is needed. This firmware implements USB serial; USB keyboard/gamepad emulation is not implemented.

Implemented: input acquisition, contact debounce, joystick RGB feedback, individual relative rotation events, shaft-button events, and serial snapshots. Game action assignments, PC application integration, Wi-Fi/Bluetooth, persistent settings, robot control, and a motion watchdog are outside the current implementation. The shaft button has no assigned game action.

| Signal | ESP32 connection | Meaning |
| --- | --- | --- |
| Joystick UP | GPIO26 | Actual player handle UP; red LED |
| Joystick DOWN | GPIO27 | Actual player handle DOWN; green LED |
| Joystick RIGHT | GPIO25 | Actual player handle RIGHT; blue LED |
| Joystick LEFT | GPIO21 | Actual player handle LEFT; white LED |
| Joystick common | GND | No joystick VCC connection |
| Encoder CLK / A | GPIO32 | Quadrature bit 1 |
| Encoder DT / B | GPIO33 | Quadrature bit 0 |
| Encoder SW | GPIO23 | Shaft press, active LOW |
| Encoder + / GND | 3V3 / GND | Shared ground; power at 3.3 V |
| Onboard WS2812 | GPIO16 | Joystick feedback; separate from the GPIO2 LED |

All seven signal inputs use `INPUT_PULLUP`. A closed joystick contact or SW connects its input to GND, reads LOW, and is reported as `true`. The allocation avoids UART0, flash pins, boot-strapping pins, input-only GPIOs without internal pull-ups, and board LED pins. See [Espressif GPIO restrictions](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/gpio.html).

## Input behavior

### Joystick and shaft button

The main loop checks the joystick and SW on a **1 ms schedule**. Each contact must remain unchanged for **10 ms** before its new level is accepted. These are software intervals, not a measured end-to-end latency guarantee.

Each accepted joystick state change emits one full four-direction `joystick` record, including release. Holding a direction or remaining neutral produces no repeated events. Diagonals retain both directions; contacts closing at different times can produce intermediate states. Opposing contacts are also preserved for diagnosis.

The RGB LED uses the same debounced joystick state. Neutral is off; UP red, DOWN green, RIGHT blue, LEFT white. Vertical directions take priority on diagonals. The complete priority is **UP > DOWN > RIGHT > LEFT**, including contradictory contacts. Each enabled channel has brightness **32/255**. Encoder rotation and SW do not change the LED.

SW is debounced independently and emits one `button` record per accepted press or release. Startup waits for the joystick and SW to settle, then sends `ready` and a combined `state`. Held inputs appear in that state without fabricated press events.

### Rotary encoder

`CHANGE` interrupts on CLK and DT capture phase and `millis()` in a fixed **128-entry edge queue**. ISR/main-loop access shares an ESP32 critical section. The ISR does not print or allocate. The loop drains at most 128 samples per pass and decodes valid one-bit Gray-code transitions in order.

Current calibration: **four edges per step**, resting phase **3 / binary 11**, **direction inversion enabled**. CW is clockwise when looking at the knob from its shaft end and reports `delta: 1`; CCW reports `delta: -1`. The owner accepted the corrected direction. One electrical step is not yet a quantitatively verified one-to-one tactile-click count.

Duplicate levels, retraced bounce, and incomplete reversals do not produce completed steps. Invalid two-bit jumps discard partial progress and resynchronize without a separate error message. Rotation does not use the 10 ms contact filter. Completed CW then CCW movements remain two events, not a net-zero sum.

Rotation during startup settling is discarded. Starting between rest phases requires synchronization before a subsequent full step can count. See the [encoder guide](dino-controller-encoder.md) for phase sequences and calibration.

## Serial and resource bounds

**115200 baud, 8N1, no flow control**. Output is compact ASCII JSON, one object per LF-terminated line. Plain-text `STATE` with LF or CRLF requests a combined snapshot. The [protocol reference](dino-controller-protocol.md) is the current wire contract.

| Configuration in `config.h` | Current value |
| --- | --- |
| `kProtocolVersion` | 0 |
| `kScanMs` / `kDebounceMs` | 1 / 10 ms |
| `kEncoderRestAB` / `kEncoderEdgesPerStep` / `kEncoderInvert` | 3 / 4 / true |
| `kEncoderEdgeCapacity` | 128 captured edges |
| `kEventCapacity` | 64 queued messages, plus the active serialized line |
| `kLineBytes` | 256-byte buffer including C terminator; frame length <256 bytes including LF |
| `kCommandBytes` | 32-byte buffer; at most 31 bytes stored before LF |
| `kRxBytesPerLoop` | At most 32 received bytes processed per loop |

UART output uses available capacity in partial writes. Either queue overflowing finishes the active JSON line, discards uncertain queued history, then reports `event_overflow` and a fresh combined state. Missing rotations cannot be reconstructed. The sketch has no `delay()` calls or heartbeat.

## Firmware layout

| File under `src/dino-controller/` | Responsibility |
| --- | --- |
| `dino-controller.ino` | Setup, scanning, LED output, serial integration |
| `config.h` | GPIOs, timings, brightness, calibration, buffer bounds |
| `joystick.h` / `joystick_led.h` | Contact debounce / direction color priority |
| `encoder.h` / `encoder_button.h` | Quadrature decoding / shaft-button debounce |
| `encoder_capture.h` | ISR edge queue and synchronized loop access |
| `serial_protocol.h` | `STATE` parser, message queue, JSON serialization |

The sketch folder and `.ino` basename must both be `dino-controller`. The implementation uses Arduino-ESP32 APIs and fixed storage, with no additional Arduino libraries. Native tests are under `tests/dino_controller/`. Keep configuration in `config.h`, hardware-independent logic testable, and each authored file at or below 300 lines.

## Implementation milestones

| Stage | Recorded result on 2026-09-19 |
| --- | --- |
| 1. Red blink | Compiled, uploaded, visually accepted; commit `120c321` |
| 2. Joystick and RGB | Actual handle-to-GPIO mapping corrected and accepted; commit `89cdaac` |
| 3. Encoder and SW | Implemented, tested, built, uploaded; owner accepted operation after `kEncoderInvert=true` |

The initial v1 proposal is superseded here by the actual v0 documentation. A future version change requires an explicit firmware/receiver compatibility decision. Detailed continuity records, exact detent counts, maximum speed, final cable noise, and latency remain [unmeasured](dino-controller-validation.md).

## Hardware references

- [Freenove board repository and pinout](https://github.com/Freenove/Freenove_ESP32_WROOM_Board), [Freenove Arduino setup](https://docs.freenove.com/projects/fnk0090/en/latest/fnk0090/codes/C/Preface.html)
- [ESP32-WROOM-32E/32UE datasheet](https://documentation.espressif.com/esp32-wroom-32e_esp32-wroom-32ue_datasheet_en.html), [Arduino GPIO API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/gpio.html), [Serial API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/serial.html)
- Owner-selected products: [board B0C9THDPXP](https://www.amazon.co.jp/dp/B0C9THDPXP), [encoder B08BBYMR9V](https://www.amazon.co.jp/dp/B08BBYMR9V), [joystick B0FBKXP82C](https://www.amazon.co.jp/dp/B0FBKXP82C). Product IDs identify purchases; connector order is established separately.
