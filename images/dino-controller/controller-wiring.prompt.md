<!-- images/dino-controller/controller-wiring.prompt.md: Record the original-image composition and pixel-preservation check for the current wiring diagram. -->
# Wiring Illustration Source Record

Updated 2026-09-19. The current [PNG diagram](controller-wiring.png) is rendered from a self-contained [SVG composition](controller-wiring.svg). It directly embeds the owner's source images. All board annotations are outside the original board rectangle.

## Preserved ESP32 board

- Source: [Freenove-ESP32-Dev-Board.png](Freenove-ESP32-Dev-Board.png), 1814 × 681 pixels.
- Source SHA-256: `7390bbf9b6ddf0db2e7b4095066ed2adee43a420af2b375e6783a1194aad6ff5`.
- Crop: x=1030, y=117, width=276, height=556, in source pixel coordinates.
- Display scale: exactly 2x in both dimensions using ImageMagick Point (nearest-neighbor) sampling, producing 552 × 1112 pixels.
- Position in the 2000 × 1450 diagram: x=850, y=170.
- The source board is retained with its original antenna, shield, header positions, labels, LEDs, buttons, and USB connector. Each source pixel becomes a 2 × 2 block.
- Verification: extract the final PNG board rectangle and compare against the enlarged source crop with ImageMagick `compare -metric AE`. Result: **0 differing pixels**.
- The full source pinout file remains unchanged. The current diagram uses direct image composition; it does not require an image-generation prompt.

## Component references and connections

The SVG also embeds [rotary-encoder-reference.png](rotary-encoder-reference.png) and [joystick-connector-reference.png](joystick-connector-reference.png) with their aspect ratios preserved. The joystick image is explicitly marked as an unverified connector reference.

Signal assignments match `src/dino-controller/config.h`: UP26, DOWN27, RIGHT25, LEFT21, CLK32, DT33, SW23. Encoder supply is 3V3; both common returns are GND. GPIO14 and 5V have no connection callouts. Leader lines align with the original header rows and stop outside the board image.

To export again from the repository root with librsvg installed:

```sh
rsvg-convert images/dino-controller/controller-wiring.svg \
  -o images/dino-controller/controller-wiring.png
```

Use the [wiring guide](../../docs/dino-controller-wiring.md) for the physical position table, player-direction mapping, and continuity procedure.
