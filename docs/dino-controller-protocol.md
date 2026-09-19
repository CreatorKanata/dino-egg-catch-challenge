<!-- docs/dino-controller-protocol.md: Specify the actual v0 JSON stream and STATE command for PC interoperability. -->
# Dino Controller JSON Protocol v0

As implemented on **2026-09-19** in [serial_protocol.h](../src/dino-controller/serial_protocol.h) and [dino-controller.ino](../src/dino-controller/dino-controller.ino), including `kEncoderInvert=true`. The protocol version remains **0** after owner acceptance of the corrected encoder operation.

## Transport and framing

| Property | Current contract |
| --- | --- |
| Transport | Board USB–UART bridge, Arduino `Serial` / UART0 |
| Serial settings | **115200 baud, 8 data bits, no parity, 1 stop bit, no flow control** |
| Device → PC | One compact JSON object per line, terminated by LF (`0x0A`) |
| Encoding | ASCII; also valid UTF-8 |
| Frame buffer | 256 bytes including C terminator; an emitted frame is at most 255 bytes including LF |
| PC → device | Plain-text `STATE` with LF or CRLF; not a JSON request |
| Periodic output | None; unchanged inputs are not repeated |

Firmware sends no human-readable application logs. ESP32 ROM boot text can precede JSON after reset. A serial read may contain part of a line or several lines: buffer until LF, then parse each complete object. A receiver may strip a trailing CR. Bound receiver buffering and discard an oversized line through its next LF.

### Common fields on every message

| Field | JSON type / range | Meaning |
| --- | --- | --- |
| `v` | Integer, exactly `0` | Current protocol version |
| `type` | String | `ready`, `state`, `joystick`, `encoder`, `button`, or `error` |
| `seq` | Integer, 0–4294967295 | Serialization/transmission sequence; starts at 0 after firmware reset; wraps modulo 2^32 |
| `ms` | Integer, 0–4294967295 | Device `millis()` timestamp; wraps after about 49.7 days |

`seq` increments when a message is serialized, not when an input is captured. It is not an acknowledgment, an input count, or proof of delivery. Discarded queued events never receive sequence numbers, so **overflow can occur without a sequence gap**.

`ms` is the acceptance time for joystick/button events, captured final-edge time for an encoder step, snapshot/startup time for state/ready, or recovery time for an error. Encoder edges may wait in a queue while other inputs are polled. Do not assume globally monotonic timestamps, sort the stream by `ms`, or compare device time directly with a PC clock. Preserve transmission order; captured encoder steps retain their own order unless overflow occurs.

## Device messages

Examples form an illustrative sequence, not a physical capture. Each object is one wire line followed by LF. Field order is shown as emitted but should not matter to a parser. Booleans are JSON `true`/`false`, not numbers or strings.

### `ready`: configuration at startup

Emitted once after initial joystick/SW debounce settles, followed by `state`. Connecting a terminal does not itself emit `ready` unless it resets the board.

```json
{"v":0,"type":"ready","seq":0,"ms":20,"device":"dino-controller","pins":{"up":26,"down":27,"left":21,"right":25},"encoder":{"clk":32,"dt":33,"sw":23,"edges_per_step":4,"rest_ab":3,"inverted":true}}
```

| Payload | Type | Meaning |
| --- | --- | --- |
| `device` | String, `dino-controller` | Firmware identity, not a unique hardware serial number |
| `pins` | Object of integer GPIOs | `up`, `down`, `left`, `right` map actual handle directions |
| `encoder` | Object | Wiring and decoder configuration below |
| `encoder.clk`, `.dt`, `.sw` | Integer GPIOs | Currently 32, 33, 23 |
| `encoder.edges_per_step` | Integer | Currently 4; configured decoding step size |
| `encoder.rest_ab` | Integer, 0–3 | Currently 3; configured resting phase |
| `encoder.inverted` | Boolean | Currently true; corrects observed CW/CCW polarity |

### `state`: complete input snapshot

Emitted after `ready`, for each accepted `STATE` command, and after overflow recovery.

```json
{"v":0,"type":"state","seq":1,"ms":20,"device":"dino-controller","up":false,"down":false,"left":false,"right":false,"button":false,"ab":3}
```

| Payload | Type | Meaning |
| --- | --- | --- |
| `device` | String, `dino-controller` | Identity for synchronization |
| `up`, `down`, `left`, `right` | Boolean | Full latest accepted joystick state |
| `button` | Boolean | Latest accepted shaft-button state; true = pressed |
| `ab` | Integer, 0–3 | Raw phase `(CLK << 1) \| DT`, sampled for this snapshot |

`ab`: 0=LOW/LOW, 1=LOW/HIGH, 2=HIGH/LOW, 3=HIGH/HIGH, in CLK/DT order. It is a diagnostic electrical level, not a rotation count, debounced step, or direction. The two GPIO reads are not an atomic phase trace. Joystick/SW fields are debounced, so bouncing contacts may differ from raw electrical levels.

Replace cached joystick/button levels with the snapshot. No absolute encoder position or missed-rotation history is provided.

### `joystick`: accepted direction change

Sent once when the combined debounced state changes, including release. All four fields are always present; replace the entire cached joystick state. This does not change the shaft-button state.

```json
{"v":0,"type":"joystick","seq":2,"ms":150,"up":true,"down":false,"left":false,"right":true}
{"v":0,"type":"joystick","seq":3,"ms":390,"up":false,"down":false,"left":false,"right":false}
```

Two adjacent true fields indicate a diagonal. Contradictory directions are preserved. Different contact settling times can produce intermediate reports. Holding a direction does not repeat events. LED color priority does not remove any serial fields.

### `encoder`: one completed electrical step

```json
{"v":0,"type":"encoder","seq":4,"ms":470,"direction":"cw","delta":1}
{"v":0,"type":"encoder","seq":5,"ms":520,"direction":"ccw","delta":-1}
```

| Payload | Type / values | Meaning |
| --- | --- | --- |
| `direction` | String, `cw` or `ccw` | Viewed from the knob's shaft end |
| `delta` | Integer, exactly 1 or -1 | `cw` pairs with 1; `ccw` pairs with -1 |

An event represents a qualified four-edge step under the current calibration, not each GPIO edge. Exact correspondence to tactile clicks needs a counted physical test. Opposite steps remain separate events even if both occur before the main loop drains its queue. There is no acceleration, batching, or accumulated-position field.

### `button`: accepted shaft press or release

```json
{"v":0,"type":"button","seq":6,"ms":630,"pressed":true}
{"v":0,"type":"button","seq":7,"ms":710,"pressed":false}
```

`pressed` is boolean: true for press, false for release. It refers to encoder SW, not a joystick button. It uses 10 ms debounce and does not repeat while held. In a combined `state`, the same accepted level is named `button`.

### `error`: event history lost

```json
{"v":0,"type":"error","seq":8,"ms":900,"code":"event_overflow"}
{"v":0,"type":"state","seq":9,"ms":900,"device":"dino-controller","up":false,"down":false,"left":false,"right":false,"button":false,"ab":3}
```

`code` is `event_overflow`, the only implemented error code. It covers the 128-entry raw-edge queue and the 64-entry message queue, with no loss count or source identifier. Invalid quadrature transitions resynchronize without emitting this error.

Recovery finishes any partially transmitted line, discards uncertain queued messages, then serializes an error and fresh combined snapshot before subsequent messages. Raw-edge overflow also discards captured history and resynchronizes the decoder. Further overload can trigger recovery again. The snapshot restores held levels but cannot reconstruct rotation.

## Host command: `STATE`

Send these exact bytes:

```text
53 54 41 54 45 0A       = STATE followed by LF
53 54 41 54 45 0D 0A    = STATE followed by CRLF
```

In a terminal, type uppercase `STATE` and append a newline; do not send the literal characters backslash and `n`. Include no spaces. The response is a `state` queued with other accepted events, subject to overflow recovery. It does not reset the board, sequence, or decoder. There is no echo, request ID, acknowledgment, or JSON command parser.

The parser stores at most **31 non-LF bytes**, including an optional CR, in a 32-byte buffer. A 32nd non-LF byte drops the command through the next LF. At LF it removes one trailing CR, then requires exactly the five bytes `STATE`. Lowercase, spaces, embedded NUL, unknown commands, and overlong lines are silently ignored. CR alone does not complete a command. RX processing starts after startup announcement and is limited to 32 bytes per loop.

## Startup and PC receiver guidance

The following guides a future PC receiver; no PC/game receiver is implemented in the firmware directory.

1. Open the correct port. Clear cached held inputs and partial line data. Opening the port may reset the board through DTR/RTS.
2. Ignore boot text and incomplete/malformed lines while synchronizing. Send `STATE` with newline; retry after boot if needed. Do not require a `ready` that may have been sent before opening.
3. Require a fresh valid **v0 combined `state`** with `device: dino-controller` before applying input. While waiting, do not apply input events to the game.
4. Replace joystick levels on `joystick`, button level on `button`, and both on `state`. Apply valid `encoder.delta` once in arrival order. Validate field types and direction/delta agreement.
5. Treat a new `ready` as a fresh firmware session. Track sequence modulo 2^32. A gap, malformed known frame, or `event_overflow` makes history uncertain: clear held inputs and obtain a fresh state. Never invent missed rotation.
6. Clear held inputs on disconnect and require a fresh snapshot on reopen. Unknown additional fields/types may be ignored after accounting for a valid common envelope; unsupported versions require explicit handling.

Startup held inputs are included in the initial state without fabricated press events. Rotation during initial settling is ignored. Silence is normal for unchanged inputs and does not prove connection health. A future receiver may poll `STATE` with application-specific timeouts. This firmware has no motion-safety watchdog or heartbeat.

## Compatibility history

- Stage 2 (`89cdaac`) also used v0, but startup and `STATE` returned joystick-only snapshots. These are insufficient for a current receiver's combined-state synchronization.
- Stage 3 retains v0 and adds combined `state`, encoder/button events, and configuration in `ready`.
- Previously proposed v1 examples are not emitted. A future v1 migration must update firmware, tests, and receiver expectations together.

See [development and debugging](dino-controller-development.md) for terminal setup and [validation](dino-controller-validation.md) for hardware evidence.
