// src/dino-controller/encoder_button.h: Debounce the active-low shaft button independently.
#pragma once
#include "config.h"

class EncoderButton {
 public:
  void begin(bool pressed, uint32_t now) {
    stable_ = candidate_ = pressed;
    changedAt_ = now;
    ready_ = false;
  }
  bool update(bool pressed, uint32_t now) {
    if (pressed != candidate_) {
      candidate_ = pressed;
      changedAt_ = now;
    }
    if (static_cast<uint32_t>(now - changedAt_) < Config::kDebounceMs) return false;
    const bool changed = !ready_ || stable_ != candidate_;
    stable_ = candidate_;
    ready_ = true;
    return changed;
  }
  bool ready() const { return ready_; }
  bool pressed() const { return stable_; }

 private:
  bool stable_ = false, candidate_ = false, ready_ = false;
  uint32_t changedAt_ = 0;
};
