// src/dino-controller/encoder_capture.h: Capture ordered A/B edges; keep decoding and UART outside ISR.
#pragma once
#include <Arduino.h>
#include "config.h"

struct EncoderEdge {
  uint32_t ms;
  uint8_t ab;
};

namespace EncoderCapture {
inline portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
inline EncoderEdge edges[Config::kEncoderEdgeCapacity]{};
inline size_t head = 0, count = 0;
inline bool overflow = false;

inline uint8_t ARDUINO_ISR_ATTR readAB() {
  return (digitalRead(Config::kEncoderClkPin) == HIGH ? 2 : 0) |
         (digitalRead(Config::kEncoderDtPin) == HIGH ? 1 : 0);
}

inline void ARDUINO_ISR_ATTR onChange() {
  portENTER_CRITICAL_ISR(&mux);
  if (count == Config::kEncoderEdgeCapacity) {
    overflow = true;
  } else if (!overflow) {
    const size_t tail = (head + count) % Config::kEncoderEdgeCapacity;
    edges[tail].ab = readAB();
    edges[tail].ms = millis();
    ++count;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

// Initialize or recover at a sampled phase under the same lock used by the ISR.
inline uint8_t reset() {
  portENTER_CRITICAL(&mux);
  head = count = 0;
  overflow = false;
  const uint8_t ab = readAB();
  portEXIT_CRITICAL(&mux);
  return ab;
}

// On overflow discard uncertain edges and return a new synchronization phase.
inline bool pop(EncoderEdge& edge, bool& lost) {
  portENTER_CRITICAL(&mux);
  lost = overflow;
  if (lost) {
    head = count = 0;
    overflow = false;
    edge = {millis(), readAB()};
  }
  const bool available = count != 0;
  if (available) {
    edge = edges[head];
    head = (head + 1) % Config::kEncoderEdgeCapacity;
    --count;
  }
  portEXIT_CRITICAL(&mux);
  return available;
}
}  // namespace EncoderCapture
