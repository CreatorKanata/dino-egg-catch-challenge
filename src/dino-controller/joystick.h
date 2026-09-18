// src/dino-controller/joystick.h: Debounce independent contacts without hardware or blocking delays.
#pragma once
#include "config.h"

class JoystickInputs {
 public:
  void begin(uint8_t pressedMask, uint32_t now) {
    state_ = pressedMask;
    ready_ = false;
    for (size_t i = 0; i < Config::kInputCount; ++i) {
      contacts_[i] = {static_cast<bool>(pressedMask & (1U << i)),
                      static_cast<bool>(pressedMask & (1U << i)), now};
    }
  }

  // Returns true for the first stable snapshot, then once per combined change.
  bool update(uint8_t pressedMask, uint32_t now) {
    uint8_t next = 0;
    bool allSettled = true;
    for (size_t i = 0; i < Config::kInputCount; ++i) {
      Contact& contact = contacts_[i];
      const bool raw = (pressedMask & (1U << i)) != 0;
      if (raw != contact.candidate) {
        contact.candidate = raw;
        contact.changedAt = now;
      }
      // Unsigned elapsed-time arithmetic handles the millis() wraparound.
      const bool settled = static_cast<uint32_t>(now - contact.changedAt) >=
                           Config::kDebounceMs;
      if (settled) contact.stable = contact.candidate;
      allSettled = allSettled && settled;
      if (contact.stable) next |= 1U << i;
    }
    const bool wasReady = ready_;
    ready_ = ready_ || allSettled;
    const bool changed = next != state_;
    state_ = next;
    return ready_ && (!wasReady || changed);
  }
  bool ready() const { return ready_; }
  uint8_t state() const { return state_; }

 private:
  struct Contact {
    bool stable = false;
    bool candidate = false;
    uint32_t changedAt = 0;
  };
  Contact contacts_[Config::kInputCount]{};
  uint8_t state_ = 0;
  bool ready_ = false;
};
