<!-- docs/dino-controller.md: Define a reviewable hardware and serial contract before Arduino firmware is implemented. -->
# Dino Controller Specification

Draft v0.1 — 2026-09-19. Stage 1 red blinking was compiled, uploaded, and visually confirmed by the owner on 2026-09-19. The current sketch implements Stage 2 joystick diagnostics with direction-dependent RGB feedback. Joystick GPIO assignments follow the owner's observed handle directions, and the owner confirmed correct operation after remapping on 2026-09-19. Stage 2 basic operation is accepted; extended physical checks remain separately recorded. Encoder handling and the final v1 protocol remain planned for Stage 3.

## 1. Scope and evidence

The user's selected hardware is an ESP32-WROOM-32E Freenove development board, a rotary encoder labeled `GND`, `+`, `SW`, `DT`, `CLK`, and a five-wire arcade joystick. The requested transport is USB serial to the connected PC. Joystick state changes and clockwise/counterclockwise rotation must be reported.

| Evidence | What it establishes | Still unverified |
| --- | --- | --- |
| User's board photo and [repository pinout](../images/dino-controller/Freenove-ESP32-Dev-Board.png) | Board layout, GPIO labels, onboard LED assignments | Actual module suffix, flash size, current wiring |
| User's encoder photo | Five printed terminal labels | Detents per revolution, transitions per detent, rest phase, pull-up circuit |
| User's joystick reference image | A diagram labeled **HORI HAYABUSA**, showing GND / DOWN / UP / RIGHT / LEFT | Applicability to the purchased joystick, connector viewing side, installed orientation |
| Freenove FNK0090 documentation | ESP32-WROOM-32E board family, CH340 bridge, Arduino board selection | Exact revision of the user's board |

Purchase references: [board B0C9THDPXP](https://www.amazon.co.jp/dp/B0C9THDPXP), [encoder B08BBYMR9V](https://www.amazon.co.jp/dp/B08BBYMR9V), [joystick B0FBKXP82C](https://www.amazon.co.jp/dp/B0FBKXP82C). Amazon page contents were unavailable during this review; no electrical rating or connector order is inferred from the product IDs.

Implementation proceeds through the three stages below, with a physical check before the next stage. Later stages cover input acquisition, diagnostics, and serial reporting. Game mappings, host GUI, Wi-Fi/Bluetooth, USB keyboard/gamepad emulation, motor control, and persistent settings are outside the initial firmware scope. The encoder switch is included as an input, without assigning it to Catch.

## Implementation stages

| Stage | Implementation | Completion check |
| --- | --- | --- |
| 1. Red LED blink | `rgbLedWrite` drives the onboard WS2812 on GPIO16; red brightness 32/255, on for 1000 ms and off for 1000 ms | Compile, upload, then visually confirm at least five red/off cycles |
| 2. Joystick | Verify common and four directions; add debounced directional state, USB serial messages, and direction-dependent RGB feedback | Confirm neutral, each direction, release, hold, diagonals, and LED colors on the actual wiring |
| 3. Rotary encoder | Confirm 3.3 V signals and SW wiring; calibrate rest phase, detents, and CW direction; add ordered rotation and switch events | Confirm per-detent counts, both directions, push/release, and simultaneous joystick operation |

Complete and review each stage's physical result before enabling the next. The current Stage 2 sketch reads the four joystick inputs; encoder pins are not configured or sampled. The final protocol includes both input devices, but Stage 2 must not report an unconnected encoder switch as a measured input: initially use `ready` and `joystick` reports plus an initial joystick report, and introduce the final combined `state` contract in Stage 3.

During Stage 2, label the staged protocol as `v: 0` (experimental). On `STATE`, reply with the current four-direction `joystick` record, including at startup. Do not label this reduced interface as protocol v1. Stage 3 enables the complete v1 schema below once the encoder switch is connected and validated. Game integration should wait for the final v1 interface.

### Stage 2 direction feedback and diagnostics

The GPIO16 WS2812 now shows the accepted joystick state instead of blinking. Brightness is 32/255 for each active color channel. Neutral is off; UP is red; DOWN is green; RIGHT is blue; LEFT is white. **Vertical directions take priority on diagonals**, as selected by the owner: UP+LEFT/RIGHT is red, DOWN+LEFT/RIGHT is green. The full deterministic priority is UP, DOWN, RIGHT, LEFT. Opposing contacts are still reported faithfully over serial so wiring faults remain visible.

`joystick.h` independently debounces each contact for 10 ms, scanning on a 1 ms schedule. The LED and serial report use the same accepted state. Startup waits for a stable baseline, then emits a v0 `ready` record (including the GPIO map) and one `joystick` snapshot. Holding or remaining neutral produces no repeated events. `STATE` with LF or CRLF requests a current `joystick` snapshot without resetting the board.

`serial_protocol.h` uses a 64-event queue and bounded 256-byte output / 32-byte command buffers. UART writes use available capacity, so input scanning continues during partial transmission. If the queue overflows, finish the active line, discard uncertain queued history, then send a v0 `error` with `code: event_overflow` and the current `joystick` state. Invalid/overlong commands are ignored through their newline. Stage 2 reports no encoder button or rotation fields. The sketch uses no `delay` calls.

The ordinary GPIO2 LED is separate from this RGB indicator. See the [sketch README](../src/dino-controller/README.md) for Serial Monitor examples and the physical direction/color checklist.

## 2. Hardware connection

```text
Joystick contacts ─┐
Encoder A/B/SW ────┼─> ESP32 GPIO ─> UART0 ─> onboard USB–UART bridge ─> PC
Common ground ────┘
```

Use the board USB connector for power and serial communication. Arduino `Serial` uses UART0 on this target. No additional USB–UART adapter or wiring to TX/RX is required. The classic ESP32-WROOM-32E does not provide native USB HID through this connector. [F1][E2][E4]

### GPIO allocation

Use the **GPIO number printed on the board**, not a physical header position. Joystick assignments below follow the owner's observations on the existing wiring (2026-09-19); encoder assignments remain proposed for Stage 3.

| Device terminal / logical signal | ESP32 connection | Configuration |
| --- | --- | --- |
| Encoder `GND` | GND | Shared ground |
| Encoder `+` | 3V3 | 3.3 V supply |
| Encoder `CLK` / A | GPIO32 | `INPUT_PULLUP`, both-edge interrupt |
| Encoder `DT` / B | GPIO33 | `INPUT_PULLUP`, both-edge interrupt |
| Encoder `SW` | GPIO23 | `INPUT_PULLUP`, active LOW |
| Joystick common, after continuity verification | GND | Shared ground |
| Joystick UP contact | GPIO26 | `INPUT_PULLUP`, active LOW |
| Joystick DOWN contact | GPIO27 | `INPUT_PULLUP`, active LOW |
| Joystick LEFT contact | GPIO21 | `INPUT_PULLUP`, active LOW |
| Joystick RIGHT contact | GPIO25 | `INPUT_PULLUP`, active LOW |

The first diagnostic mapping reported DOWN for physical UP, LEFT for DOWN, RIGHT for LEFT, and UP for RIGHT. The owner's handle direction is authoritative: correct the GPIO names in `config.h` and retain the existing wiring. Serial fields and LED colors share this corrected map.

Each joystick switch should connect its signal to common only while actuated. The intended passive joystick requires no VCC wire. Inactive inputs read HIGH; active contacts read LOW. In the protocol, `true` means active.

Power the encoder at **3.3 V**, including any onboard pull-ups. Do not connect the encoder `+` to 5 V: a pull-up to that supply could place 5 V on ESP32 inputs. Verify the purchased module works at 3.3 V before connecting its signal wires. [E4]

The allocation avoids GPIO0/2/5/12/15 strapping pins, UART0 GPIO1/3, flash GPIO6–11, GPIO34–39 without internal pull-ups, and board LED GPIO2/16. GPIO21/23 are available here because no I2C/SPI peripherals are enabled. Start with internal pull-ups; evaluate external 4.7–10 kΩ pull-ups to 3V3 only if measured noise or cable length requires them. [E1][E3][F2]

The candidate joystick order `GND, DOWN, UP, RIGHT, LEFT` is **not an approved wiring diagram for this unit**. Follow [the continuity procedure](dino-controller-validation.md) before assigning pins. Wire colors and a mirrored connector drawing are insufficient evidence.

## 3. Input behavior

### Joystick and push switch

- Scan the four joystick contacts and encoder push switch on a nonblocking schedule; proposed scan interval: 1 ms.
- Debounce each contact independently. Accept a new level after it remains unchanged for 10 ms; a new raw transition restarts that input's timer.
- On each scan, update all accepted joystick levels, then emit one `joystick` message if the combined state changed. Include all four directions, including releases.
- Support two-contact diagonals. Do not require diagonal contacts to close at exactly the same instant; intermediate states may be reported.
- Report physical states even if UP+DOWN or LEFT+RIGHT are both true. Device firmware does not silently select a winner; the host can flag invalid combinations.
- Emit one `button` message per accepted encoder-switch press or release. Holding an input generates no repeat messages.
- A pulse shorter than the debounce interval may be filtered out. Timing values require physical validation.

### Rotary encoder

Define clockwise from the user's view looking down at the knob shaft. One tactile click/detent is the proposed user-visible step; individual CLK/DT edges are internal measurements, not PC events.

- Capture `CHANGE` on both CLK and DT. Decode the quadrature Gray-code sequence with a state machine; only transitions changing one bit are valid.
- Full-step decoding is the initial proposal: one complete four-transition cycle returns to a measured rest phase and emits one event. Some encoders have two transitions per detent; enable a calibrated half-step mode only after measurement.
- Initialize from the actual A/B state. Synchronize to a rest phase before counting; startup in the middle of a detent must not invent a rotation event.
- Contact bounce that retraces transitions and a reversal before completion must not generate a completed step. Invalid two-bit jumps discard the partial step and resynchronize without guessing direction.
- Calibrate direction against physical knob movement. A configuration flag can invert the decoder sign; do not assume the phase sequence labeled positive is always physical CW.
- Preserve every completed step and its order. CW followed by CCW must remain two events, not disappear into a summed counter.
- Keep interrupt handlers short: acquire/decode state and enqueue compact events. Serialize and send in the main loop; do not print, allocate strings, delay, or block in an ISR. Protect ISR/main-loop shared state.
- Do not apply the joystick's fixed 10 ms debounce window to A/B edges; it could suppress legitimate fast rotation.

Proposed bench target: correct counting at 20 detents/second, including direction changes, with no loss while operating the joystick. This is a test target, not a verified encoder capability.

## 4. Serial protocol v1

### Transport and framing

| Property | Proposed value |
| --- | --- |
| Link | Board USB serial port / UART0 |
| UART | 115200 baud, 8 data bits, no parity, 1 stop bit; no flow control |
| Device output | UTF-8/ASCII JSON object per line, terminated by LF (`\n`) |
| Maximum output line | 256 bytes including LF |
| Common fields | `v: 1`, `type`, `seq`, `ms` |
| `seq` | Unsigned 32-bit transmission sequence; starts at 0 after reset and wraps modulo 2^32 |
| `ms` | Unsigned 32-bit milliseconds since boot at event acceptance / snapshot capture; wraps modulo 2^32 |

Sequence numbers describe transmission order. Each emitted line increments `seq`; they do not acknowledge delivery. Normal output contains protocol messages only. Ignore non-JSON ROM boot text, unknown message types, and unknown additional fields; reject malformed known messages or unsupported protocol versions. A receiver must handle split reads, multiple lines per read, and an optional trailing CR. Discard an oversized/invalid line through its next newline, then resume framing.

### Device messages

Examples are illustrative, not a captured session. Each object below is one complete line.

```json
{"v":1,"type":"ready","seq":0,"ms":20,"device":"dino-controller"}
{"v":1,"type":"state","seq":1,"ms":20,"device":"dino-controller","up":false,"down":false,"left":false,"right":false,"button":false}
{"v":1,"type":"joystick","seq":2,"ms":150,"up":true,"down":false,"left":false,"right":true}
{"v":1,"type":"joystick","seq":3,"ms":390,"up":false,"down":false,"left":false,"right":false}
{"v":1,"type":"encoder","seq":4,"ms":470,"direction":"cw","delta":1}
{"v":1,"type":"encoder","seq":5,"ms":520,"direction":"ccw","delta":-1}
{"v":1,"type":"button","seq":6,"ms":630,"pressed":true}
{"v":1,"type":"button","seq":7,"ms":710,"pressed":false}
{"v":1,"type":"error","seq":8,"ms":900,"code":"event_overflow"}
{"v":1,"type":"state","seq":9,"ms":901,"device":"dino-controller","up":false,"down":false,"left":false,"right":false,"button":false}
```

| Type | Required payload and meaning |
| --- | --- |
| `ready` | `device` is `dino-controller`; emitted once per firmware startup |
| `state` | `device`, four directional booleans, and `button`; authoritative current accepted levels |
| `joystick` | Four directional booleans; replace the entire previous joystick state |
| `encoder` | `direction` is `cw` or `ccw`; `delta` must respectively be `1` or `-1`; exactly one detent |
| `button` | `pressed` boolean for the encoder push switch |
| `error` | `code: event_overflow`; one or more input events could not be retained |

No absolute encoder position is transmitted: it is a relative, endless input. A state snapshot cannot reconstruct missed rotation events.

### Startup and resynchronization

After initializing and establishing debounced input levels, emit `ready`, then `state`. An input held at power-on appears as active in this snapshot. Do not fabricate a press or a rotation solely because the device started.

The only host command in v1 is ASCII **`STATE\n`**, also accepting `STATE\r\n`. Reply with one `state` message after previously accepted events, without resetting the encoder or sequence counter. Capture the snapshot at its ordered position in the output stream so later events cannot precede an older snapshot. Bound command input to 32 bytes including the delimiter; ignore unknown commands and discard an overlong command through the next LF. Do not echo commands.

On open/reopen, the host clears cached input, discards any partial line, and sends `STATE`. It can retry after boot if the first request was lost; a CH340 port identifier alone does not uniquely identify this controller. Accept a fresh v1 `state` with the correct `device` before applying input. Opening a port can reset the ESP32 via DTR/RTS; tolerate boot text and a new `ready`. A `ready` message resets host session state regardless of its sequence value. [E5]

For normal messages, expect the next sequence modulo 2^32. A gap, invalid frame, or overflow marks input history uncertain: clear held host inputs, request a fresh state, and never synthesize missed encoder steps. On serial disconnect, clear cached inputs immediately. During synchronization, ignore events until a valid snapshot arrives.

There is no unsolicited heartbeat or repeated unchanged input report in v1. Silence is therefore **not proof of a working connection**. A later host integration may poll `STATE` for liveness and must define its own timeout before driving a robot; this document does not establish a motion-safety watchdog.

### Output buffering

Use a bounded queue, proposed capacity 64 accepted events, and bounded line buffers. Input capture must continue while serial output drains. Preserve individual rotation order and avoid silently overwriting old events. At 115200 8N1 the nominal UART budget is 11,520 bytes/second; validate sustained rates against actual serialized line lengths.

If the queue overflows, latch the fault. Finish any partially written line, invalidate queued history, report `event_overflow`, and send a newly ordered state snapshot before resuming normal events. The host must treat lost rotation as unrecoverable. Synchronize this recovery with ISR producers; never interleave bytes from different JSON messages. Use available TX capacity in chunks rather than requiring a whole 256-byte line to fit in the UART buffer.

## 5. Firmware structure and configuration

The current Stage 2 files are listed below. Encoder-specific modules will be added in Stage 3:

```text
src/dino-controller/
  dino-controller.ino  # GPIO acquisition, RGB output, and serial integration
  config.h             # GPIOs, timings, brightness, and buffer limits
  joystick.h           # Hardware-independent switch debounce
  joystick_led.h       # Direction colors and vertical priority
  serial_protocol.h    # Bounded v0 diagnostics and STATE commands
tests/dino_controller/
  test_joystick.cpp     # Production sketch tested with simulated hardware
  fakes/Arduino.h      # GPIO, clock, RGB, and bounded UART simulation
  run_tests.py         # Sanitized native build and JSON-stream assertions
```

The sketch basename must match the `dino-controller` folder. Keep each authored file at or below 300 lines. Keep debounce, quadrature, and framing logic testable without ESP32 hardware. Use Arduino-ESP32 APIs and fixed-size storage initially; no encoder or JSON library is required. [A1]

Centralize the proposed GPIO table, 115200 baud, 1 ms scan interval, 10 ms debounce interval, encoder full/half-step mode, measured rest phase, direction inversion, queue size, and line/command limits in `config.h`. Use rollover-safe elapsed-time arithmetic. If Python host tooling is added later, put its tunables in `config.py`.

A temporary diagnostic build should expose physical GPIO levels and captured A/B transitions at controlled manual speed. It is a bench aid, clearly separate from protocol-v1 output, and must be disabled in the normal build. Prefer a compile-time setting over adding more runtime commands. Finalize its format during implementation.

## 6. Open physical checks

| Item | Required evidence before acceptance |
| --- | --- |
| Current GPIO wiring | Joystick directions observed by the owner; retain this map and verify the proposed encoder allocation |
| Joystick common and order | Continuity table and connector photo with viewing orientation |
| Directions | Basic corrected operation accepted by the owner on 2026-09-19; detailed repetition and diagonal results remain unrecorded |
| Encoder electrical behavior | Confirm 3.3 V supply, idle signal levels, SW closes to GND |
| Encoder step calibration | Measure rest phase, transitions per detent, CW phase order, and total detents per turn |
| Noise, debounce, maximum speed | Test with final cable lengths and enclosure |
| Board build settings | Confirm actual module flash/PSRAM variant; initial proposal is 4 MB, no PSRAM |

These do not prevent drafting or a compile-only implementation, but no physical behavior is accepted until measured. See the [validation plan](dino-controller-validation.md).

## 7. Primary references

Reviewed 2026-09-19. Arduino APIs and CLI usage were checked with Context7 and primary documentation. Hardware statements were cross-checked against manufacturer sources and the supplied pinout.

- **[F1]** [Freenove FNK0090 preparation and Arduino setup](https://docs.freenove.com/projects/fnk0090/en/latest/fnk0090/codes/C/Preface.html)
- **[F2]** [Freenove ESP32 WROOM board repository and pinout](https://github.com/Freenove/Freenove_ESP32_WROOM_Board)
- **[E1]** [Espressif Arduino GPIO API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/gpio.html)
- **[E2]** [Espressif Arduino Serial/UART API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/serial.html)
- **[E3]** [ESP32 GPIO restrictions](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/gpio.html)
- **[E4]** [ESP32-WROOM-32E/32UE datasheet](https://documentation.espressif.com/esp32-wroom-32e_esp32-wroom-32ue_datasheet_en.html)
- **[E5]** [Espressif boot mode and automatic reset](https://docs.espressif.com/projects/esptool/en/latest/esp32/advanced-topics/boot-mode-selection.html)
- **[A1]** [Arduino sketch naming and file structure](https://github.com/arduino/arduino-cli/blob/master/docs/sketch-specification.md)
- **[A2]** [Arduino CLI getting started](https://github.com/arduino/arduino-cli/blob/master/docs/getting-started.md)
- **[A3]** [Arduino-ESP32 installation](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html)
