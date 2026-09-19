// src/dino-controller/config.h: Centralize controller wiring and provisional encoder calibration.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace Config {
constexpr uint8_t kRgbLedPin = 16;
constexpr uint8_t kLedBrightness = 32;
// Match the owner's observed handle directions on the existing wiring (2026-09-19).
constexpr uint8_t kUpPin = 26;
constexpr uint8_t kDownPin = 27;
constexpr uint8_t kLeftPin = 21;
constexpr uint8_t kRightPin = 25;
constexpr uint8_t kEncoderClkPin = 32;
constexpr uint8_t kEncoderDtPin = 33;
constexpr uint8_t kEncoderSwPin = 23;
// Confirm these on the physical encoder before promoting the protocol to v1.
constexpr uint8_t kEncoderRestAB = 3;  // A=CLK (bit 1), B=DT (bit 0): both HIGH.
constexpr uint8_t kEncoderEdgesPerStep = 4;  // Set to 2 only for measured half-step detents.
// The owner observed reversed CW/CCW on this wiring (2026-09-19).
constexpr bool kEncoderInvert = true;
constexpr size_t kEncoderEdgeCapacity = 128;
// Bit order matches each full-state serial report: up, down, left, right.
constexpr uint8_t kJoystickPins[] = {kUpPin, kDownPin, kLeftPin, kRightPin};
constexpr size_t kInputCount = sizeof(kJoystickPins) / sizeof(kJoystickPins[0]);
constexpr uint8_t kUp = 1 << 0;
constexpr uint8_t kDown = 1 << 1;
constexpr uint8_t kLeft = 1 << 2;
constexpr uint8_t kRight = 1 << 3;
constexpr uint32_t kScanMs = 1;
constexpr uint32_t kDebounceMs = 10;
constexpr unsigned long kBaud = 115200;
constexpr unsigned kProtocolVersion = 0;  // Stage 3 diagnostics; physical calibration pending.
constexpr size_t kEventCapacity = 64;
constexpr size_t kLineBytes = 256;
constexpr size_t kCommandBytes = 32;
constexpr size_t kRxBytesPerLoop = 32;
static_assert(kDebounceMs > 0 && kScanMs > 0, "Input intervals must be positive");
static_assert(kEventCapacity >= 2 && kLineBytes >= 256, "Diagnostic buffers too small");
static_assert(kEncoderRestAB < 4 &&
              (kEncoderEdgesPerStep == 2 || kEncoderEdgesPerStep == 4), "Invalid encoder calibration");
static_assert(kEncoderEdgeCapacity > 0, "Encoder queue must not be empty");
static_assert(kCommandBytes >= 7, "Command buffer must fit STATE plus CRLF");
}  // namespace Config
