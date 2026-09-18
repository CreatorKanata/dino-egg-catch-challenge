// src/dino-controller/dino-controller.ino: Verify the Freenove board with a red-only RGB blink.
#include <Arduino.h>
#include "config.h"

void setup() {
  // The WS2812 needs a color-data command; digitalWrite cannot select its color.
  rgbLedWrite(Config::kRgbLedPin, 0, 0, 0);
}

void loop() {
  rgbLedWrite(Config::kRgbLedPin, Config::kRedBrightness, 0, 0);
  delay(Config::kBlinkIntervalMs);
  rgbLedWrite(Config::kRgbLedPin, 0, 0, 0);
  delay(Config::kBlinkIntervalMs);
}
