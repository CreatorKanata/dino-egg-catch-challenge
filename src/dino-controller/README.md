<!-- src/dino-controller/README.md: Explain the first board smoke test and the planned input controller. -->
# Dino Controller

The current Arduino sketch is a board smoke test: the onboard WS2812 RGB LED lights **red for one second, then turns off for one second**, repeatedly. Green and blue remain off. Input handling and USB serial events are planned in the [detailed specification](../../docs/dino-controller.md); they are not enabled in this sketch.

## Board and wiring

- Board: Freenove ESP32-WROOM-32E development board.
- Onboard WS2812 data pin: **GPIO16**, as labeled in the [board pinout](../../images/dino-controller/Freenove-ESP32-Dev-Board.png).
- Connect the board to the PC with a USB data cable. The onboard LED requires no external wiring.
- `config.h` centralizes the pin, red brightness (32 out of 255), and 1000 ms on/off interval.

Use Arduino-ESP32's built-in `rgbLedWrite`; no additional LED library is required. This addressable RGB LED is separate from the ordinary GPIO2 LED.

## Arduino IDE

1. Open `dino-controller.ino` in this folder.
2. Select **esp32 by Espressif Systems** in Boards Manager. The installed core used for this milestone is **3.3.11**.
3. Select **ESP32 Dev Module** (`esp32:esp32:esp32`), 4 MB flash and PSRAM Disabled.
4. Run **Verify**, then choose the connected board's serial port for **Upload**.
5. Check that the RGB LED shows red for one second and is off for one second, repeating for at least five cycles. Neither green nor blue should light.

The blocking `delay` calls deliberately keep this first LED check simple. Replace them with nonblocking timing when adding controller inputs.

## Compile from the repository root

```sh
ARDUINO_CLI='/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli'
"$ARDUINO_CLI" compile \
  --fqbn esp32:esp32:esp32 \
  --board-options FlashSize=4M,PSRAM=disabled,PartitionScheme=default,DebugLevel=none \
  --warnings all --build-path "$PWD/build/dino-controller" \
  src/dino-controller
```

Verified on 2026-09-19: compilation passed with Arduino CLI 1.5.1 and Arduino-ESP32 3.3.11 for ESP32 Dev Module. Flash usage: 271,745 / 1,310,720 bytes (20%); static RAM: 22,804 / 327,680 bytes (6%). Upload passed on `/dev/cu.usbserial-110` at 115200 baud using esptool 5.3.1; the device identified as ESP32-D0WD-V3 revision 3.1. Written-data hashes were verified, and the board was reset via RTS. The project owner confirmed correct red blinking on the connected board on 2026-09-19. Stage 1 is accepted; joystick wiring verification is next.

## Implementation order

1. Complete and visually verify the red LED blink.
2. Verify joystick wiring and add directional inputs.
3. Calibrate and add rotary encoder rotation and push-switch inputs.

## Detailed specifications

- [Input hardware, GPIO allocation, debounce, and serial protocol](../../docs/dino-controller.md)
- [Joystick continuity checks, encoder calibration, and future acceptance tests](../../docs/dino-controller-validation.md)

References: [Freenove board setup](https://docs.freenove.com/projects/fnk0090/en/latest/fnk0090/codes/C/Preface.html), [Espressif BlinkRGB example](https://github.com/espressif/arduino-esp32/blob/3.3.11/libraries/ESP32/examples/GPIO/BlinkRGB/BlinkRGB.ino).
