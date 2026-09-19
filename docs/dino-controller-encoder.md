<!-- docs/dino-controller-encoder.md: Guide Stage 3 encoder calibration and document diagnostic behavior. -->
# Rotary Encoder Bring-up

2026-09-19. The owner connected the encoder after accepting Stage 2 joystick firmware (`89cdaac`). Stage 3 adds rotary and shaft-button diagnostics. After initially observing reversed CW/CCW, the owner accepted operation with direction inversion enabled. Basic operation and corrected polarity are accepted; counted detent, maximum-speed, and latency measurements remain unrecorded.

## Wiring and retained controls

See the [illustrated wiring guide](dino-controller-wiring.md) for component images, actual board pin positions, and the distinction between verified joystick directions and an unverified connector order.

| Encoder terminal | ESP32 connection |
| --- | --- |
| GND | GND, shared with the joystick |
| + | 3V3, never 5V |
| CLK / A | GPIO32 |
| DT / B | GPIO33 |
| SW | GPIO23 |

All three signals use `INPUT_PULLUP`. SW is active LOW. The verified joystick map remains UP=26, DOWN=27, LEFT=21, RIGHT=25. Its GPIO16 RGB feedback remains red/green/white/blue respectively; neutral is off and vertical directions take priority on diagonals. Encoder movement does not change the joystick LED.

## Serial Monitor procedure

Use **115200 baud, 8N1**, with newline enabled. The last verified board port was `/dev/cu.usbserial-110`; recheck the port after reconnecting USB.

1. After reset, `ready` reports joystick pins, encoder pins, and current calibration. A combined `state` follows after the joystick and SW settle. A button already held at startup appears in that snapshot without a fabricated press event.
2. Send `STATE` to obtain joystick directions, debounced `button`, and the current raw `ab` phase. `ab = (CLK << 1) | DT`: 0=LOW/LOW, 1=LOW/HIGH, 2=HIGH/LOW, 3=HIGH/HIGH. These raw levels are sampled when requested and are not a complete edge trace.
3. Push the knob, then release it. Expect one `button` event with `pressed: true`, then one with `pressed: false`. Holding it should not repeat events.
4. Looking at the knob from the shaft end, turn slowly through 10 tactile clicks clockwise, then 10 counterclockwise. Record event directions and counts separately. Polarity is owner-accepted; this counted test checks clicks per electrical cycle and should be repeated after rewiring or calibration changes.
5. Try a short clockwise/counterclockwise reversal and operate the joystick concurrently. Each completed step should remain an individual event; the joystick reports and LED should still work.

Illustrative records (not physical captures):

```json
{"v":0,"type":"encoder","seq":2,"ms":100,"direction":"cw","delta":1}
{"v":0,"type":"encoder","seq":3,"ms":200,"direction":"ccw","delta":-1}
{"v":0,"type":"button","seq":4,"ms":300,"pressed":true}
{"v":0,"type":"button","seq":5,"ms":400,"pressed":false}
{"v":0,"type":"state","seq":6,"ms":500,"device":"dino-controller","up":false,"down":false,"left":false,"right":false,"button":false,"ab":3}
```

The actual interface remains **v0**, including after basic hardware acceptance. Compared with Stage 2, startup and `STATE` now return combined `state` records instead of `joystick` snapshots. Direction changes retain the four-field `joystick` record. All message fields and recovery rules are in the [current protocol reference](dino-controller-protocol.md); a future version change requires an explicit compatibility decision.

## Calibration settings

All settings are in `src/dino-controller/config.h`:

| Setting | Current value | Adjustment after observation |
| --- | --- | --- |
| `kEncoderRestAB` | 3 (both HIGH) | Set to the measured resting phase |
| `kEncoderEdgesPerStep` | 4 | Use 2 only if each click traverses two edges and rest alternates between opposite phases |
| `kEncoderInvert` | true | Corrects reversed CW/CCW reported by the owner on this wiring |

The raw positive electrical cycle is `11 -> 10 -> 00 -> 01 -> 11`, with A=CLK and B=DT. With the corrected inversion enabled, it reports `ccw`; the reverse cycle `11 -> 01 -> 00 -> 10 -> 11` reports `cw`. Four valid transitions returning to the configured rest produce one event. This is not yet a claim of one event per physical click. If 10 clicks yield only 5 events, check the resting phase at each click with `STATE` before changing to half-step mode. Keep physical wiring fixed while calibrating firmware polarity.

## Acquisition and recovery

- `CHANGE` interrupts on CLK and DT store raw A/B samples and timestamps in a fixed 128-entry queue. ISR and loop access share an ESP32 spinlock; the ISR never prints or allocates memory.
- The loop drains at most 128 samples per pass and decodes them in order. A complete CW then CCW produces two events, not a net sum. This separates edge capture from serial backpressure.
- The decoder follows Gray-code transitions. Repeated levels, retraced contact bounce, and incomplete reversals do not produce a completed step. Invalid two-bit jumps discard partial progress and resynchronize at a rest phase. No 10 ms time filter is applied to rotation edges.
- Startup rotation during the initial input-settling period is ignored. Starting between rest phases waits for synchronization before counting a new complete step.
- SW independently uses the same 1 ms scan / 10 ms debounce as the joystick. Press and release each emit once; the LED continues to represent only joystick state.
- Decoded events enter the existing 64-entry serial queue. Available UART capacity bounds each write. `STATE` snapshots share this queue with input events.
- Overflow of either queue finishes the current partial JSON line, reports `event_overflow`, then emits a fresh combined state. Edge overflow also discards uncertain edge history and resets decoder synchronization. Missing rotation is not reconstructed.
- The diagnostic `ab` field is observational, not a synchronized encoder position. Two GPIO reads and interrupt latency can miss transitions at high rotation rates; maximum speed and cable noise still need hardware validation.

## Validation record

Direction feedback (2026-09-19): the owner initially reported CW/CCW reversed. `kEncoderInvert` was set to true without changing wiring or step size. A regression uses the configured phase sequences to require CW => +1 and CCW => -1. All eight host cases passed, Arduino compilation passed with unchanged flash/RAM usage, and upload/hash verification succeeded. A bounded `STATE` check returned all input fields false and `ab: 3` (`seq: 2`, `ms: 23342`), then closed the port. The owner subsequently accepted corrected operation. This establishes basic polarity acceptance, not a measured events-per-click or speed result.

- Eight host test cases passed under AddressSanitizer and UndefinedBehaviorSanitizer, including native assertions for 1,000 forward/reverse cycles, duplicate edges, bounce, partial reversals, invalid transitions, arbitrary startup, full/half-step decoding, polarity inversion, and alternate rest phase.
- The actual sketch is exercised with simulated interrupts, partial UART writes, held startup SW, SW bounce and timer rollover, concurrent joystick input, edge-queue overflow, and ordered recovery. Existing joystick regressions also pass. Simulation does not establish multicore timing or real contact quality.
- Arduino CLI 1.5.1 / ESP32 core 3.3.11 compilation passed for `esp32:esp32:esp32`, 4 MB flash, PSRAM disabled: 292,949 / 1,310,720 bytes flash (22%), 25,492 / 327,680 bytes static RAM (7%).
- The initial Stage 3 upload to `/dev/cu.usbserial-110` also passed; its 115200 / 8N1 `STATE` response had `seq: 2`, `ms: 17555`, all input fields false, and `ab: 3`. Both serial-check sessions closed and released the port. The later post-correction result and owner acceptance above are the current evidence.

Run `python3 tests/dino_controller/run_tests.py` from the repository root. See the [validation record](dino-controller-validation.md) for remaining measurements and the [development guide](dino-controller-development.md) for build/upload and terminal setup.

Implementation references: [Arduino GPIO interrupts](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/gpio.html), [ESP-IDF critical sections](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/freertos_idf.html#critical-sections). The installed Arduino-ESP32 3.3.11 `RepeatTimer` example also verifies the `portMUX_TYPE` and `portENTER_CRITICAL_ISR` usage.
