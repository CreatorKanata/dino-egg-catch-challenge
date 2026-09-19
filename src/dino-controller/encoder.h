// src/dino-controller/encoder.h: Decode complete Gray-code steps without time-based edge filtering.
#pragma once
#include "config.h"

class RotaryEncoder {
 public:
  RotaryEncoder(uint8_t edges = Config::kEncoderEdgesPerStep,
                uint8_t rest = Config::kEncoderRestAB, bool invert = Config::kEncoderInvert)
      : edges_(edges), rest_(rest), invert_(invert) {}

  void begin(uint8_t ab) {
    previous_ = ab;
    progress_ = 0;
    synced_ = atRest(ab);
  }

  int8_t update(uint8_t ab) {
    const uint8_t previous = previous_;
    previous_ = ab;
    if (previous == ab) return 0;
    if ((previous ^ ab) == 3) {
      begin(ab);  // A two-bit jump has unknown direction; discard the partial step.
      return 0;
    }
    if (!synced_) {
      if (atRest(ab)) begin(ab);  // Starting between detents must not create a step.
      return 0;
    }
    // Positive sequence: 11 -> 10 -> 00 -> 01 -> 11. Config maps this electrical sign to physical CW/CCW.
    const uint8_t transition = (previous << 2) | ab;
    const int8_t delta = (transition == 0xE || transition == 0x8 ||
                          transition == 0x1 || transition == 0x7) ? 1 : -1;
    progress_ += delta;
    if (!atRest(ab)) return 0;
    const int8_t step = progress_ == edges_ ? 1 : progress_ == -edges_ ? -1 : 0;
    progress_ = 0;  // Retraced bounce and partial reversals cancel at a rest position.
    return invert_ ? -step : step;
  }

 private:
  bool atRest(uint8_t ab) const {
    return ab == rest_ || (edges_ == 2 && ab == (rest_ ^ 3));
  }
  uint8_t edges_, rest_, previous_ = 0;
  bool invert_, synced_ = false;
  int8_t progress_ = 0;
};
