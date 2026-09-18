// tests/dino_controller/test_joystick.cpp: Exercise the production sketch with synthetic switch signals.
#include <assert.h>
#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include "../../src/dino-controller/dino-controller.ino"

void testDebounce() {
  JoystickInputs input;
  const uint32_t d = Config::kDebounceMs;
  input.begin(0, 0);
  assert(!input.update(0, d - 1) && !input.ready());
  assert(input.update(0, d) && input.ready());
  assert(!input.update(Config::kUp, 2 * d));
  assert(!input.update(0, 3 * d - 1));
  assert(!input.update(Config::kUp, 3 * d));
  assert(!input.update(Config::kUp, 4 * d - 1));
  assert(input.update(Config::kUp, 4 * d));
  assert(input.state() == Config::kUp && !input.update(Config::kUp, 5 * d));
  assert(!input.update(0, 6 * d));
  assert(input.update(0, 7 * d) && input.state() == 0);

  input.begin(Config::kLeft, 0);
  assert(input.update(Config::kLeft, d) && input.state() == Config::kLeft);
  input.begin(0, 0);
  input.update(0, d);
  assert(!input.update(Config::kUp, 2 * d));
  assert(!input.update(Config::kUp | Config::kRight, 2 * d + 1));
  assert(input.update(Config::kUp | Config::kRight, 3 * d));
  assert(input.state() == Config::kUp);
  assert(input.update(Config::kUp | Config::kRight, 3 * d + 1));
  assert(input.state() == (Config::kUp | Config::kRight));
  const uint32_t start = UINT32_MAX - d / 2;
  input.begin(0, start);
  assert(!input.update(Config::kDown, start));
  assert(!input.update(Config::kDown, start + d - 1));
  assert(input.update(Config::kDown, start + d));
}

void testCommandsAndQueue() {
  StateCommandReader parser;
  auto feed = [&parser](const std::string& text) {
    unsigned accepted = 0;
    for (char ch : text) accepted += parser.feed(ch);
    return accepted;
  };
  assert(feed("STA") == 0 && feed("TE\r\nSTATE\n") == 2);
  assert(feed("state\nUNKNOWN\n") == 0);
  assert(feed(std::string("STATE\0X\n", 8)) == 0);
  assert(feed(std::string(Config::kCommandBytes, 'X') + "STATE\nSTATE\n") == 1);

  SerialMessages queue;
  queue.enqueue(MessageKind::Joystick, UINT32_MAX, Config::kLeft);
  queue.prepare(0, 0);
  std::string first;
  first.append(queue.data(), 5);
  queue.consume(5);
  for (size_t i = 0; i < Config::kEventCapacity; ++i) {
    assert(queue.enqueue(MessageKind::Joystick, i, Config::kUp));
  }
  assert(!queue.enqueue(MessageKind::Joystick, 0, 0));
  queue.prepare(Config::kRight, 10);
  first.append(queue.data(), queue.remaining());
  queue.consume(queue.remaining());
  assert(first.find("\"left\":true") != std::string::npos && first.back() == '\n');
  queue.prepare(Config::kRight, 10);
  assert(std::string(queue.data(), queue.remaining()).find("event_overflow") != std::string::npos);
  queue.consume(queue.remaining());
  queue.prepare(Config::kRight, 10);
  assert(std::string(queue.data(), queue.remaining()).find("\"right\":true") != std::string::npos);
}

void testLedColors() {
  const uint8_t b = Config::kLedBrightness;
  auto check = [](uint8_t mask, uint8_t red, uint8_t green, uint8_t blue) {
    const JoystickColor color = joystickColor(mask);
    assert(color.red == red && color.green == green && color.blue == blue);
  };
  check(0, 0, 0, 0);
  check(Config::kUp, b, 0, 0);
  check(Config::kDown, 0, b, 0);
  check(Config::kRight, 0, 0, b);
  check(Config::kLeft, b, b, b);
  check(Config::kUp | Config::kRight, b, 0, 0);
  check(Config::kUp | Config::kLeft, b, 0, 0);
  check(Config::kDown | Config::kRight, 0, b, 0);
  check(Config::kDown | Config::kLeft, 0, b, 0);
  check(Config::kUp | Config::kDown, b, 0, 0);
}

void pump(uint32_t time) {
  fakeMillis = time;
  for (unsigned i = 0; i < 300; ++i) loop();
}

void testObservedPinMapping() {
  // Literal GPIOs are the owner's physical observations, independent of Config's map.
  const uint8_t b = Config::kLedBrightness;
  struct ObservedDirection {
    uint8_t pin, state, red, green, blue;
  };
  const ObservedDirection observed[] = {
      {26, Config::kUp, b, 0, 0}, {27, Config::kDown, 0, b, 0},
      {21, Config::kLeft, b, b, b}, {25, Config::kRight, 0, 0, b}};
  for (const auto& direction : observed) {
    for (int& pin : fakePins) pin = HIGH;
    fakePins[direction.pin] = LOW;
    assert(readJoystick() == direction.state);
    const JoystickColor color = joystickColor(readJoystick());
    assert(color.red == direction.red && color.green == direction.green &&
           color.blue == direction.blue);
  }
  for (int& pin : fakePins) pin = HIGH;
  fakePins[26] = fakePins[25] = LOW;  // Physical UP + RIGHT.
  assert(readJoystick() == (Config::kUp | Config::kRight));
  const JoystickColor diagonal = joystickColor(readJoystick());
  assert(diagonal.red == b && diagonal.green == 0 && diagonal.blue == 0);
}

void setMask(uint8_t mask) {
  for (size_t i = 0; i < Config::kInputCount; ++i) {
    fakePins[Config::kJoystickPins[i]] = (mask & (1U << i)) ? LOW : HIGH;
  }
}

void settleMask(uint8_t mask, uint32_t time) {
  const uint8_t oldRed = fakeRed, oldGreen = fakeGreen, oldBlue = fakeBlue;
  setMask(mask);
  pump(time);
  assert(fakeRed == oldRed && fakeGreen == oldGreen && fakeBlue == oldBlue);
  pump(time + Config::kDebounceMs);
  const JoystickColor color = joystickColor(mask);
  assert(fakeRed == color.red && fakeGreen == color.green && fakeBlue == color.blue);
}

void testSketch() {
  for (int& pin : fakePins) pin = HIGH;
  setMask(Config::kLeft);  // Test a contact already held at power-on.
  setup();
  assert(fakeLedPin == Config::kRgbLedPin);
  assert(fakeRed == 0 && fakeGreen == 0 && fakeBlue == 0);
  for (uint8_t pin : Config::kJoystickPins) assert(fakeModes[pin] == INPUT_PULLUP);
  assert(Serial.baud == Config::kBaud);
  pump(Config::kDebounceMs - 1);
  assert(Serial.output.empty());
  pump(Config::kDebounceMs);
  assert(fakeRed == Config::kLedBrightness && fakeGreen == Config::kLedBrightness &&
         fakeBlue == Config::kLedBrightness);
  const size_t startupSize = Serial.output.size();
  pump(100);
  assert(Serial.output.size() == startupSize);
  settleMask(0, 101);
  settleMask(Config::kUp, 201);
  settleMask(Config::kDown, 301);
  settleMask(Config::kRight, 401);
  settleMask(Config::kLeft, 501);
  settleMask(Config::kUp | Config::kRight, 601);
  settleMask(Config::kUp | Config::kDown, 701);  // Preserve opposing physical states.
  settleMask(0, 801);
  // Snapshot requests neither alter held state nor fabricate extra presses.
  Serial.receive("STATE\r\n");
  pump(900);
  Serial.receive("STATE\n");
  loop();
  Serial.capacity = 0;
  for (size_t i = 0; i <= Config::kEventCapacity; ++i) {
    settleMask(i % 2 == 0 ? Config::kUp : 0, 1000 + i * (Config::kDebounceMs + 1));
  }
  Serial.capacity = 13;
  pump(3000);
  std::cout << Serial.output;  // Parsed as JSON by the Python runner.
}

int main() {
  testDebounce();
  testCommandsAndQueue();
  testLedColors();
  testObservedPinMapping();
  testSketch();
}
