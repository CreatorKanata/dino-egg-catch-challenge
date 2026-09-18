// src/dino-controller/serial_protocol.h: Bound diagnostic commands and queue complete JSON frames.
#pragma once
#include <stdio.h>
#include <string.h>
#include "config.h"

class StateCommandReader {
 public:
  bool feed(char ch) {
    if (ch == '\n') {
      if (size_ && buffer_[size_ - 1] == '\r') --size_;
      const bool matched = !dropping_ && size_ == 5 && memcmp(buffer_, "STATE", 5) == 0;
      size_ = 0;
      dropping_ = false;
      return matched;
    }
    if (!dropping_) {
      if (size_ + 1 >= sizeof(buffer_)) dropping_ = true;
      else buffer_[size_++] = ch;
    }
    return false;
  }

 private:
  char buffer_[Config::kCommandBytes]{};
  size_t size_ = 0;
  bool dropping_ = false;
};

enum class MessageKind : uint8_t { Ready, Joystick, Overflow };
struct Message {
  MessageKind kind;
  uint32_t ms;
  uint8_t state;
};

class SerialMessages {
 public:
  bool enqueue(MessageKind kind, uint32_t now, uint8_t state = 0) {
    if (overflow_ || count_ == Config::kEventCapacity) {
      overflow_ = true;
      return false;
    }
    queue_[(head_ + count_) % Config::kEventCapacity] = {kind, now, state};
    ++count_;
    return true;
  }

  void prepare(uint8_t currentState, uint32_t now) {
    if (remaining()) return;  // Finish a partially transmitted line before recovery.
    if (overflow_) {
      head_ = count_ = 0;
      overflow_ = false;
      enqueue(MessageKind::Overflow, now);
      enqueue(MessageKind::Joystick, now, currentState);
    }
    if (count_ == 0) return;
    const Message message = queue_[head_];
    head_ = (head_ + 1) % Config::kEventCapacity;
    --count_;
    const char* type = message.kind == MessageKind::Ready ? "ready" :
                       message.kind == MessageKind::Joystick ? "joystick" : "error";
    const int prefix = snprintf(line_, sizeof(line_),
        "{\"v\":%u,\"type\":\"%s\",\"seq\":%lu,\"ms\":%lu",
        Config::kProtocolVersion, type, static_cast<unsigned long>(sequence_),
        static_cast<unsigned long>(message.ms));
    if (prefix < 0 || static_cast<size_t>(prefix) >= sizeof(line_)) return;
    char* tail = line_ + prefix;
    const size_t room = sizeof(line_) - prefix;
    int written = 0;
    if (message.kind == MessageKind::Ready) {
      written = snprintf(tail, room,
          ",\"device\":\"dino-controller\",\"pins\":{\"up\":%u,\"down\":%u,\"left\":%u,\"right\":%u}}\n",
          static_cast<unsigned>(Config::kUpPin), static_cast<unsigned>(Config::kDownPin),
          static_cast<unsigned>(Config::kLeftPin), static_cast<unsigned>(Config::kRightPin));
    } else if (message.kind == MessageKind::Joystick) {
      written = snprintf(tail, room,
          ",\"up\":%s,\"down\":%s,\"left\":%s,\"right\":%s}\n",
          boolean(message.state & Config::kUp), boolean(message.state & Config::kDown),
          boolean(message.state & Config::kLeft), boolean(message.state & Config::kRight));
    } else {
      written = snprintf(tail, room, ",\"code\":\"event_overflow\"}\n");
    }
    if (written < 0 || static_cast<size_t>(written) >= room) return;
    length_ = static_cast<size_t>(prefix + written);
    offset_ = 0;
    ++sequence_;
  }

  const char* data() const { return line_ + offset_; }
  size_t remaining() const { return length_ - offset_; }
  void consume(size_t bytes) {
    if (bytes <= remaining()) offset_ += bytes;
  }

 private:
  static const char* boolean(bool value) { return value ? "true" : "false"; }
  Message queue_[Config::kEventCapacity]{};
  size_t head_ = 0;
  size_t count_ = 0;
  bool overflow_ = false;
  char line_[Config::kLineBytes]{};
  size_t length_ = 0;
  size_t offset_ = 0;
  uint32_t sequence_ = 0;
};
