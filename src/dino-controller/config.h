// src/dino-controller/config.h: Keep the board LED pin, brightness and blink timing together.
#pragma once
#include <stdint.h>

namespace Config {
constexpr uint8_t kRgbLedPin = 16;  // Freenove onboard WS2812, according to its pinout.
constexpr uint8_t kRedBrightness = 32;
constexpr uint32_t kBlinkIntervalMs = 1000;
}  // namespace Config
