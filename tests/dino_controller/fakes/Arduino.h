// tests/dino_controller/fakes/Arduino.h: Simulate GPIO, time, RGB and partial UART writes for tests.
#pragma once
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <deque>
#include <string>

constexpr int LOW = 0;
constexpr int HIGH = 1;
constexpr int INPUT_PULLUP = 2;
constexpr int SERIAL_8N1 = 3;
constexpr int CHANGE = 4;
#define ARDUINO_ISR_ATTR
struct portMUX_TYPE { unsigned depth = 0; };
#define portMUX_INITIALIZER_UNLOCKED {}
inline void portENTER_CRITICAL(portMUX_TYPE* mux) { assert(mux->depth++ == 0); }
inline void portEXIT_CRITICAL(portMUX_TYPE* mux) { assert(--mux->depth == 0); }
inline void portENTER_CRITICAL_ISR(portMUX_TYPE* mux) { portENTER_CRITICAL(mux); }
inline void portEXIT_CRITICAL_ISR(portMUX_TYPE* mux) { portEXIT_CRITICAL(mux); }
inline void (*fakeInterrupts[40])(){};
inline void attachInterrupt(uint8_t pin, void (*handler)(), int mode) {
  assert(mode == CHANGE);
  fakeInterrupts[pin] = handler;
}
inline uint32_t fakeMillis = 0;
inline int fakePins[40]{};
inline int fakeModes[40]{};
inline uint8_t fakeLedPin = 0;
inline uint8_t fakeRed = 0;
inline uint8_t fakeGreen = 0;
inline uint8_t fakeBlue = 0;
inline uint32_t millis() { return fakeMillis; }
inline void pinMode(uint8_t pin, int mode) { fakeModes[pin] = mode; }
inline int digitalRead(uint8_t pin) { return fakePins[pin]; }
inline void rgbLedWrite(uint8_t pin, uint8_t red, uint8_t green, uint8_t blue) {
  fakeLedPin = pin;
  fakeRed = red;
  fakeGreen = green;
  fakeBlue = blue;
}
inline void yield() {}

struct FakeSerial {
  unsigned long baud = 0;
  int capacity = 13;  // Force every frame to span several loop iterations.
  std::string output;
  std::deque<char> input;
  void begin(unsigned long value, int framing) {
    assert(framing == SERIAL_8N1);
    baud = value;
  }
  int availableForWrite() const { return capacity; }
  size_t write(const uint8_t* bytes, size_t size) {
    assert(size <= static_cast<size_t>(capacity));
    output.append(reinterpret_cast<const char*>(bytes), size);
    return size;
  }
  int available() const { return static_cast<int>(input.size()); }
  int read() {
    if (input.empty()) return -1;
    const char ch = input.front();
    input.pop_front();
    return ch;
  }
  void receive(const std::string& bytes) {
    for (char ch : bytes) input.push_back(ch);
  }
};
inline FakeSerial Serial;
