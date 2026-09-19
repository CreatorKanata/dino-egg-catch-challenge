<!-- src/dino-controller/README.md: Start development of the verified ESP32 joystick and rotary input controller. -->
# Dino Controller

Arduino firmware for the Freenove ESP32-WROOM-32E board, arcade joystick, and rotary encoder with shaft switch. The owner accepted basic joystick operation and corrected CW/CCW rotation on **2026-09-19**. The current USB serial protocol remains **v0**.

![Current controller wiring](../../images/dino-controller/controller-wiring.png)

The HAYABUSA inset is an unverified connector reference. Joystick GPIO assignments describe actual player handle directions, not a presumed five-pin order.

| Input | ESP32 connection | Behavior |
| --- | --- | --- |
| Joystick UP / DOWN | GPIO26 / GPIO27 | Red / green |
| Joystick RIGHT / LEFT | GPIO25 / GPIO21 | Blue / white |
| Joystick common | GND | Passive contacts; no VCC |
| Encoder CLK / DT / SW | GPIO32 / GPIO33 / GPIO23 | Relative rotation / shaft press |
| Encoder + / GND | 3V3 / GND | 3.3 V only |

The onboard GPIO16 WS2812 is off at neutral. Vertical inputs take color priority on diagonals; serial preserves both directions. Joystick/SW scan every 1 ms with 10 ms debounce. Encoder decoding uses four edges per step, rest `ab=3`, and `kEncoderInvert=true`; CW reports +1 and CCW -1.

## Quick start

1. Check the [illustrated wiring](../../docs/dino-controller-wiring.md).
2. Open `dino-controller.ino` in Arduino IDE. Select **ESP32 Dev Module**, **esp32 by Espressif Systems 3.3.11**, 4 MB flash, PSRAM Disabled, default partition, Core Debug Level None.
3. Follow the [development guide](../../docs/dino-controller-development.md) to test, build, and upload. Close/disconnect serial monitors before uploading.
4. Connect one terminal at **115200 baud, 8N1, no flow control**. Send uppercase `STATE` with LF or CRLF for a combined snapshot. Check joystick directions, knob rotation, and shaft press/release.
5. Disconnect the terminal when finished so the USB serial port is available to others.

No additional Arduino libraries are needed. Run native tests from the repository root:

```sh
python3 tests/dino_controller/run_tests.py
```

## Documentation

- [Current implementation and configuration](../../docs/dino-controller.md)
- [Illustrated wiring and original pinout](../../docs/dino-controller-wiring.md)
- [IDE/CLI build, upload, CoolTerm, and troubleshooting](../../docs/dino-controller-development.md)
- [Complete JSON v0 message protocol and STATE command](../../docs/dino-controller-protocol.md)
- [Encoder calibration and decoding](../../docs/dino-controller-encoder.md)
- [Accepted stages, test evidence, and remaining measurements](../../docs/dino-controller-validation.md)

Stage 1 red blinking is preserved in commit `120c321`; Stage 2 joystick/RGB in `89cdaac`. The current Stage 3 adds encoder/SW and combined snapshots. Eight native test cases and the last Arduino build/upload passed. Exact tactile-click counts, maximum speed, and latency have not been measured; see the validation record.
