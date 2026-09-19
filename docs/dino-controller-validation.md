<!-- docs/dino-controller-validation.md: Separate accepted hardware behavior, software evidence, and unmeasured physical checks. -->
# Controller Validation Record

Updated **2026-09-19** after owner acceptance of the corrected encoder direction. Basic operation of all three stages is accepted. This does not imply measured detent counts, maximum speed, or latency. Build/upload instructions are maintained in the [development guide](dino-controller-development.md), with actual messages in the [v0 protocol reference](dino-controller-protocol.md).

## Completed implementation stages

| Stage | Implementation / physical evidence |
| --- | --- |
| 1. Red LED | GPIO16 WS2812, brightness 32/255, one second red / one second off. Build/upload passed and owner confirmed blinking. Commit `120c321`. The current firmware replaces blinking with direction feedback |
| 2. Joystick | Owner identified actual UP=26, DOWN=27, LEFT=21, RIGHT=25. Firmware names were corrected while retaining wires. Owner confirmed correct basic operation after upload. Commit `89cdaac` |
| 3. Encoder | Owner connected CLK32, DT33, SW23, 3V3/GND and confirmed operation, initially reporting CW/CCW reversed. `kEncoderInvert=true` corrected polarity. After rebuild/upload, owner confirmed correct operation |

Initial joystick reports were DOWN for actual UP, LEFT for DOWN, RIGHT for LEFT, and UP for RIGHT. The current map is based on actual handle direction, not the supplied HAYABUSA connector reference. See the [illustrated wiring](dino-controller-wiring.md).

Stage 2 used joystick-only v0 snapshots. Stage 3 uses combined v0 `state` records. Hardware acceptance has not promoted the version to v1.

## Recorded software, build, and upload evidence

| Check | Recorded result on 2026-09-19 |
| --- | --- |
| Stage 2 native tests | Four host cases passed, including calibrated GPIO direction and LED assertions |
| Stage 2 compile | 289,325 / 1,310,720 bytes flash; 23,940 / 327,680 bytes static RAM |
| Stage 3 native tests after polarity correction | Eight cases passed under AddressSanitizer and UndefinedBehaviorSanitizer, including the configured CW/CCW regression |
| Stage 3 compile | CLI 1.5.1, ESP32 core 3.3.11, `esp32:esp32:esp32`, 4 MB flash, PSRAM disabled; 292,949 / 1,310,720 bytes flash (22%), 25,492 / 327,680 bytes static RAM (7%) |
| Stage 3 upload | `/dev/cu.usbserial-110`, flash hashes verified, board reset completed |
| Serial check after polarity correction | Bounded 115200 / 8N1 session sent `STATE`, received the combined response below, then closed the port |
| Owner follow-up | Accepted operation after the CW/CCW correction |

Captured response from the post-correction serial check:

```json
{"v":0,"type":"state","seq":2,"ms":23342,"device":"dino-controller","up":false,"down":false,"left":false,"right":false,"button":false,"ab":3}
```

This capture verifies the combined response and sampled inactive levels, not mechanical direction or exact step counts. The later owner confirmation provides basic physical acceptance. CoolTerm was disconnected for the upload; the checking script closed its handle afterward. Port ownership must be checked anew for future sessions.

## Hardware-independent coverage

Run `python3 tests/dino_controller/run_tests.py` from the repository root. Tests compile production logic and the actual sketch with simulated GPIO/time/RGB/UART/interrupts; they do not establish real electrical timing or contact quality.

| Area | Coverage |
| --- | --- |
| Joystick | Independent 10 ms debounce, bounce, release, held startup, diagonals, opposing contacts, actual GPIO mapping, timer rollover |
| LED | Neutral off, four colors, vertical priority, shared accepted state |
| Decoder | 1,000 forward/reverse synthetic cycles; duplicate/bouncing edges, incomplete reversal, invalid jumps, arbitrary startup phase |
| Calibration | Four/two-edge modes, inversion, alternate rest phase, current configured CW/+1 and CCW/-1 |
| SW | Bounce, press/release, held startup, timer rollover |
| Integration | Both rotation signs captured before loop drain, simultaneous joystick operation, simulated interrupts |
| Protocol | JSON payloads, LF/CRLF `STATE`, invalid/bounded commands, partial writes, message and edge overflow recovery |

## Joystick continuity worksheet

This is still a documentation gap for the physical harness, even though functional GPIO mapping works.

1. Disconnect USB and the joystick harness from the board. Test only the unpowered joystick.
2. Photograph the connector, latch, viewing side, and installed player orientation. Assign P1–P5 in that photo.
3. Find which pair becomes continuous for each actual handle direction. The shared wire is common; neutral should open all directional contacts.
4. Check supported diagonals and release. If no common wire is shared by all four directions, inspect the switches before assuming this connector layout.
5. Record wire IDs below. Never infer them from the unrelated HAYABUSA image or harness colors alone.

| Handle movement | Common / directional wire ID | Current verified GPIO |
| --- | --- | --- |
| UP | ___ / ___ | 26 |
| DOWN | ___ / ___ | 27 |
| LEFT | ___ / ___ | 21 |
| RIGHT | ___ / ___ | 25 |
| Neutral | All open: ___ | No active input |
| Supported diagonal | Common + two wires: ___ | Both corresponding inputs |

## Extended physical checks still to record

The following counts and thresholds are **proposed test criteria**, not reported measurements. Operate only the input controller/serial receiver for these checks; robot movement is outside this milestone.

| Check | Proposed criterion / evidence | Status |
| --- | --- | --- |
| Neutral/held startup | `ready` then correct combined `state`, no fabricated press/rotation | Dedicated record pending |
| Four directions | 20 press/release cycles each, correct fields/colors, no duplicates | Basic accepted; counted run pending |
| Diagonals | 10 cycles per supported diagonal; both fields, vertical color priority | Dedicated record pending |
| Shaft button | 20 press/release cycles, one event per transition | Basic operation accepted; counted run pending |
| Slow rotation | 20 tactile clicks each direction; compare event count and resting phases | Direction accepted; exact count pending |
| Detent calibration | Rest phase at consecutive clicks, edges per click, detents per revolution | Pending |
| Fast rotation | Controlled 20 clicks/second for 5 seconds each direction, compare 100 events | Proposed target; not measured |
| Concurrent inputs/reversal | Preserve separate CW/CCW while joystick and SW change | Simulated; dedicated physical record pending |
| Idle | No unsolicited input records over 60 seconds | Dedicated record pending |
| Reconnect/reset | Recover held state through `STATE`; tolerate boot text and new session | Combined response captured; full scenarios pending |
| Disconnect handling | Future PC receiver clears held inputs and resynchronizes on reopen | Host implementation pending |
| Queue recovery | Explicit error, complete lines, fresh snapshot under injected load | Simulated; hardware stress test pending |
| Latency | Proposed ≤30 ms stable input/step completion to complete PC frame | Not measured |
| Final wiring | Supply levels, cable noise, installed enclosure, connector photo | Detailed electrical record pending |

Measure rate/count and latency with a repeatable fixture or signal capture before making quantitative claims. Device `ms` is not synchronized to the PC clock. Electrical steps are not automatically proven identical to tactile clicks.

## What to save for future changes

Record source revision, IDE/CLI/core versions, FQBN/options, GPIO and calibration values, cable/enclosure setup, build result, test results, connector photos, and complete serial captures. Distinguish simulation, compilation, upload, basic owner acceptance, and measured results. Record serial disconnection at session end.
