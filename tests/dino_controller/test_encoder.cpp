// tests/dino_controller/test_encoder.cpp: Exercise decoding, shaft debounce and interrupt integration.
#include <assert.h>
#include <initializer_list>
#include <iostream>
#include <string>
#include <vector>
#include "../../src/dino-controller/dino-controller.ino"

std::vector<int> feed(RotaryEncoder& decoder, std::initializer_list<uint8_t> phases) {
  std::vector<int> steps;
  for (uint8_t ab : phases) {
    const int step = decoder.update(ab);
    if (step) steps.push_back(step);
  }
  return steps;
}

void testDecoder() {
  RotaryEncoder decoder(4, 3, false);  // Test the electrical decoder independently of calibration.
  decoder.begin(3);
  assert(feed(decoder, {3, 2, 2, 0, 1, 3, 3}) == std::vector<int>{1});
  assert(feed(decoder, {1, 0, 2, 3}) == std::vector<int>{-1});
  assert(feed(decoder, {2, 3, 2, 0, 2, 0, 1, 3, 1, 3}) == std::vector<int>{1});
  assert(feed(decoder, {2, 0, 2, 3}).empty());  // Reverse before completing a step.
  assert(feed(decoder, {2, 1, 3}).empty());  // Invalid two-bit transition resynchronizes.
  assert(feed(decoder, {2, 0, 1, 3}) == std::vector<int>{1});
  for (uint8_t value : {0, 1, 2}) {
    decoder.begin(value);
    assert(feed(decoder, {3}).empty());
    assert(feed(decoder, {2, 0, 1, 3}) == std::vector<int>{1});
  }
  RotaryEncoder half(2, 3, false);
  half.begin(3);
  assert((feed(half, {2, 0, 1, 3}) == std::vector<int>{1, 1}));
  assert((feed(half, {1, 0, 2, 3}) == std::vector<int>{-1, -1}));
  RotaryEncoder inverted(4, 3, true);
  inverted.begin(3);
  assert(feed(inverted, {2, 0, 1, 3}) == std::vector<int>{-1});
  RotaryEncoder otherRest(4, 0, false);
  otherRest.begin(0);
  assert(feed(otherRest, {1, 3, 2, 0}) == std::vector<int>{1});
  decoder.begin(3);
  for (unsigned i = 0; i < 1000; ++i) {
    assert((feed(decoder, {2, 0, 1, 3, 1, 0, 2, 3}) == std::vector<int>{1, -1}));
  }
}

void testConfiguredDirection() {
  // Observed physical CW previously produced ccw; keep this regression independent of the flag.
  RotaryEncoder calibrated;
  calibrated.begin(3);
  assert(feed(calibrated, {1, 0, 2, 3}) == std::vector<int>{1});
  assert(feed(calibrated, {2, 0, 1, 3}) == std::vector<int>{-1});
}

void testButton() {
  EncoderButton button;
  const uint32_t start = UINT32_MAX - 5;
  button.begin(true, start);
  assert(!button.update(true, start + Config::kDebounceMs - 1));
  assert(button.update(true, start + Config::kDebounceMs));
  assert(button.ready() && button.pressed());
  assert(!button.update(false, 20));
  assert(!button.update(true, 25));
  assert(!button.update(false, 30));
  assert(button.update(false, 40) && !button.pressed());
  assert(!button.update(false, 50));
}

void pump(uint32_t ms) {
  fakeMillis = ms;
  for (unsigned i = 0; i < 300; ++i) loop();
}

void phase(uint8_t ab, uint32_t ms) {
  fakeMillis = ms;
  const bool clkChanged = fakePins[32] != ((ab & 2) ? HIGH : LOW);
  const bool dtChanged = fakePins[33] != ((ab & 1) ? HIGH : LOW);
  fakePins[32] = (ab & 2) ? HIGH : LOW;
  fakePins[33] = (ab & 1) ? HIGH : LOW;
  if (clkChanged) fakeInterrupts[32]();
  if (dtChanged) fakeInterrupts[33]();
}

void testSketch() {
  for (int& pin : fakePins) pin = HIGH;
  fakePins[23] = LOW;  // A held shaft button appears in the initial state, not a new press.
  setup();
  for (uint8_t pin : {32, 33, 23}) assert(fakeModes[pin] == INPUT_PULLUP);
  assert(fakeInterrupts[32] && fakeInterrupts[33]);
  pump(10);
  const size_t idle = Serial.output.size();
  pump(50);
  assert(Serial.output.size() == idle);
  // Two opposite steps before loop runs must survive as two events, not a net zero.
  for (uint8_t ab : {2, 0, 1, 3}) phase(ab, 100);
  for (uint8_t ab : {1, 0, 2, 3}) phase(ab, 101);
  fakePins[26] = LOW;  // Simultaneous physical UP still lights red.
  pump(110);
  pump(120);
  assert(fakeRed == Config::kLedBrightness && fakeGreen == 0 && fakeBlue == 0);
  fakePins[23] = HIGH; pump(130);
  fakePins[23] = LOW; pump(135);
  fakePins[23] = HIGH; pump(140); pump(150);
  fakePins[23] = LOW; pump(160); pump(170);
  Serial.receive("STATE\n"); pump(180);
  // Overflow during a partially sent frame must preserve framing and report lost history.
  Serial.capacity = 1;
  Serial.receive("STATE\n"); fakeMillis = 190; loop();
  for (size_t i = 0; i <= Config::kEncoderEdgeCapacity; ++i) phase(i % 2 ? 3 : 2, 200);
  pump(210);
  Serial.capacity = 13; pump(220);
  phase(3, 230); pump(230);  // Re-synchronize before a complete new cycle.
  for (uint8_t ab : {2, 0, 1, 3}) phase(ab, 240);
  pump(250);
  std::cout << Serial.output;
}

int main() {
  testDecoder();
  testConfiguredDirection();
  testButton();
  testSketch();
}
