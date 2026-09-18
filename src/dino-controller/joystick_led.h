// src/dino-controller/joystick_led.h: Map confirmed directions to the user's diagnostic LED colors.
#pragma once
#include "config.h"

struct JoystickColor {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

inline JoystickColor joystickColor(uint8_t state) {
  // Vertical directions take priority on diagonals; serial still reports every bit.
  // Impossible opposing contacts remain visible in serial; this order is deterministic.
  if (state & Config::kUp) return {Config::kLedBrightness, 0, 0};
  if (state & Config::kDown) return {0, Config::kLedBrightness, 0};
  if (state & Config::kRight) return {0, 0, Config::kLedBrightness};
  if (state & Config::kLeft) {
    return {Config::kLedBrightness, Config::kLedBrightness, Config::kLedBrightness};
  }
  return {0, 0, 0};
}
