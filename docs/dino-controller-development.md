<!-- docs/dino-controller-development.md: Provide reproducible build, upload, and debugging steps for future development. -->
# Controller Development and Debugging

Working environment recorded on **2026-09-19**. Start with [illustrated wiring](dino-controller-wiring.md), then use the [v0 protocol reference](dino-controller-protocol.md) when inspecting output.

## Known working target

| Setting | Value |
| --- | --- |
| Hardware | Freenove ESP32-WROOM-32E development board |
| Arduino IDE / bundled CLI | 2.3.10 / 1.5.1 on the development Mac |
| Boards Manager package | **esp32 by Espressif Systems 3.3.11** |
| Arduino board | **ESP32 Dev Module** |
| FQBN | `esp32:esp32:esp32` |
| Flash Size | 4MB (`FlashSize=4M`) |
| PSRAM | Disabled (`PSRAM=disabled`) |
| Partition Scheme | Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS), `PartitionScheme=default` |
| Core Debug Level | None (`DebugLevel=none`) |
| Upload Speed | 115200 (`UploadSpeed=115200`) |
| Other options | Installed board defaults; record later changes |
| Additional Arduino libraries | None; RGB support comes from the ESP32 core |
| Last verified macOS port | `/dev/cu.usbserial-110`; enumerate again after reconnecting |
| Observed USB bridge | CH340, VID `1A86`, PID `7523` |

Select the classic ESP32 target above. Keep the recorded core baseline for reproducibility; changing versions requires rebuilding and checking hardware behavior. The flash/PSRAM options are the tested configuration for this board.

## Arduino IDE: setup, build, and write the board

1. Install Arduino IDE. Add Espressif's stable Boards Manager URL in settings if needed: `https://espressif.github.io/arduino-esp32/package_esp32_index.json`.
2. Install **esp32 by Espressif Systems 3.3.11** in Boards Manager. See the [official installation guide](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html).
3. Open `src/dino-controller/dino-controller.ino` in its matching folder beside the headers.
4. Select **ESP32 Dev Module** and the options above. Connect a USB data cable and select the actual board port. A CH340 port with an unrecognized board name can still use this manually selected target.
5. Click **Verify** and resolve errors. This compiles; it does not write the board.
6. Disconnect CoolTerm and close other serial monitors. Click **Upload** and wait for successful writing/verification and reset. A successful compile alone is not an upload result.
7. Open one terminal at **115200 / 8N1 / no flow control**. Send `STATE` with newline and check the combined snapshot. Compare input behavior with the [validation checklist](dino-controller-validation.md).
8. Disconnect the terminal afterward so the owner or another tool can use the port.

This board needs no custom USB CDC/HID options or separate TX/RX adapter. Upload resets the firmware and sequence counter. Upload speed and runtime baud are independent settings; both currently happen to be 115200.

## Arduino CLI: reproducible commands

Run from the repository root in one shell. On this Mac, the IDE bundles a CLI that is not assumed to be on PATH:

```sh
ARDUINO_CLI='/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli'
DINO_BUILD_DIR="$PWD/build/dino-controller"
"$ARDUINO_CLI" version
"$ARDUINO_CLI" core list
"$ARDUINO_CLI" board details --fqbn esp32:esp32:esp32
"$ARDUINO_CLI" board list
```

On another machine, set the installed executable path, or `ARDUINO_CLI=arduino-cli` when available on PATH. For a clean core installation:

```sh
"$ARDUINO_CLI" core update-index \
  --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
"$ARDUINO_CLI" core install esp32:esp32@3.3.11 \
  --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
```

Run tests, then compile current source:

```sh
python3 tests/dino_controller/run_tests.py
"$ARDUINO_CLI" compile \
  --fqbn esp32:esp32:esp32 \
  --board-options FlashSize=4M,PSRAM=disabled,PartitionScheme=default,DebugLevel=none \
  --warnings all \
  --build-path "$DINO_BUILD_DIR" \
  src/dino-controller
```

Proceed only after both succeed. Inspect `board list`, confirm the controller port, and release it from monitoring applications. Replace this example port if enumeration differs:

```sh
DINO_PORT='/dev/cu.usbserial-110'
lsof -n "$DINO_PORT"
```

On macOS, `lsof` shows processes holding the port; no listed process normally means it is available. Close/disconnect the owning application, then write the successful build:

```sh
"$ARDUINO_CLI" upload \
  --fqbn esp32:esp32:esp32 \
  --port "$DINO_PORT" \
  --board-options UploadSpeed=115200 \
  --input-dir "$DINO_BUILD_DIR" \
  src/dino-controller
```

**`upload` does not compile.** Never upload a stale build after compilation fails or after editing source. `--input-dir` above matches compile's `--build-path`. Outputs are in ignored `build/`. Commands were checked against CLI 1.5.1 help; see the [official upload reference](https://github.com/arduino/arduino-cli/blob/master/docs/commands/arduino-cli_upload.md).

## CoolTerm and Arduino Serial Monitor

Use only one port owner: CoolTerm, Arduino Serial Monitor, CLI monitor, or a custom script. A terminal left open can keep the port occupied.

| Setting | Value |
| --- | --- |
| Port | Currently enumerated controller port |
| Baud | 115200 |
| Data bits / parity / stop bits | 8 / None / 1 |
| Hardware/software flow control | None; disable RTS/CTS and XON/XOFF |
| Send line ending | LF or CRLF |
| Local echo | Optional display setting; firmware does not echo |

In **CoolTerm**, choose these options and Connect. Use text/line sending to send uppercase `STATE` followed by LF or CRLF. Expect `type: state`, four joystick booleans, `button`, and `ab`. Click **Disconnect** afterward and verify disconnected status before handing the port back.

In **Arduino Serial Monitor**, select 115200 and Newline or Both NL & CR, then send `STATE`. CR alone does not trigger a reply. Close the monitor before switching to CoolTerm or uploading with another tool.

Opening the port can reset the ESP32 through DTR/RTS. Request a snapshot if startup output was missed; neutral and held inputs intentionally produce no repeated events. ROM boot text is expected after reset. See [Espressif's boot-mode reference](https://docs.espressif.com/projects/esptool/en/latest/esp32/advanced-topics/boot-mode-selection.html).

### Optional CLI monitor

After closing other terminals:

```sh
"$ARDUINO_CLI" monitor \
  --port "$DINO_PORT" \
  --config baudrate=115200
```

Exit with Ctrl+C, wait for the shell prompt, and check `lsof -n "$DINO_PORT"` if ownership is unclear. Use CoolTerm or the IDE for explicit send-line-ending control. Display timestamps, if enabled separately, are PC receipt times; JSON `ms` is ESP32 time. See [CLI monitor options](https://github.com/arduino/arduino-cli/blob/master/docs/commands/arduino-cli_monitor.md).

## A useful debugging session

1. Record firmware revision, core version, and port. Connect and request `STATE`.
2. Check neutral: all directions false, `button: false`, normally `ab: 3` at rest. Disconnected inputs also appear inactive, so this alone does not prove wiring.
3. Move UP/DOWN/RIGHT/LEFT and release each. Expect red/green/blue/white respectively, then off. Serial direction must match actual handle movement.
4. Test supported diagonals: both fields true; UP wins red and DOWN wins green. Holding a contact should not repeat messages.
5. Rotate CW then CCW: expect `direction: cw, delta: 1` then `direction: ccw, delta: -1`. Press/release SW: expect `pressed: true` then `false`.
6. Count tactile clicks and events separately for exact calibration; follow the [encoder procedure](dino-controller-encoder.md). Save representative complete JSON lines.
7. Disconnect and record that the port was released. Scripts must close serial handles in a `finally` block or context manager, including on errors.

## Troubleshooting

| Symptom | Check / next action |
| --- | --- |
| Port missing | USB data cable, connector/hub, power, OS enumeration; compare `board list` before/after reconnecting |
| Board shown as Unknown | Manually select ESP32 Dev Module and the verified port; CH340 identity is not an FQBN |
| Port busy | Disconnect CoolTerm, close monitors/scripts, inspect `lsof`, then retry |
| Upload stays at Connecting | Check target/port and USB. If auto-reset fails, hold BOOT during connection; tap/release EN/RST while holding BOOT, then release BOOT when writing starts |
| Stuck in download mode | Release BOOT, then press/release EN/RST to run the application |
| Garbled output | Set 115200, 8N1, no flow control; inspect complete lines |
| No idle output | Send exact `STATE` plus LF/CRLF; input events are change-only |
| All directions remain false | Check common GND and continuity; disconnected pull-up inputs also read HIGH |
| Wrong direction/color | Follow actual player handle direction and verified GPIOs; reference connector labels may differ |
| CW/CCW reversed after rewiring | Check CLK32/DT33 and `kEncoderInvert=true`; record observations before changing polarity |
| One event per two clicks | Measure resting `ab` at consecutive clicks and edges per click before considering two-edge mode |
| Button noisy or stuck | Check SW23, GND, wiring, stable levels; inspect snapshots and change events |
| `event_overflow` | History was lost. Read recovery state, inspect noisy A/B wiring and sustained event/TX load; do not infer missing rotations |
| LED dark at neutral | Expected; GPIO16 direction feedback replaced the Stage 1 blink |

Manual BOOT/EN operation follows [Espressif's boot guide](https://docs.espressif.com/projects/esptool/en/latest/esp32/advanced-topics/boot-mode-selection.html). Further upload diagnosis: [esptool troubleshooting](https://docs.espressif.com/projects/esptool/en/latest/esp32/troubleshooting.html).

## Development workflow and test limits

Make one behavior change at a time, put tunables in `config.h`, add a meaningful native regression test, and update docs. Run `python3 tests/dino_controller/run_tests.py`, then Arduino compilation, then a focused upload/physical check when needed. Release serial afterward.

Native tests need Python 3 and a C++17 compiler with AddressSanitizer and UndefinedBehaviorSanitizer; this Mac uses Clang. No extra PyPI packages are required. They execute production logic and the sketch with simulated GPIO, clock, RGB, interrupts, and bounded UART writes. They do not establish switch quality, real ISR timing, or maximum speed.

Keep debug text out of JSON and Core Debug Level at None for protocol checks. Avoid printing, delays, and allocation in the ISR. Use a logic analyzer or separate diagnostic build for detailed electrical timing; `STATE.ab` is an occasional sample, not a transition trace.

Record build output, board options, revision, wiring/calibration, and physical results in the [validation record](dino-controller-validation.md). The last Stage 3 build used 292,949 / 1,310,720 bytes flash and 25,492 / 327,680 bytes static RAM; later builds may differ.
