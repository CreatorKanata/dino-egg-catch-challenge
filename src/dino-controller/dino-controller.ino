// src/dino-controller/dino-controller.ino: Verify joystick wiring with USB reports and direction colors.
#include <Arduino.h>
#include "config.h"
#include "joystick.h"
#include "joystick_led.h"
#include "serial_protocol.h"

JoystickInputs joystick;
StateCommandReader commands;
SerialMessages messages;
uint32_t lastScan = 0;
bool announced = false;

uint8_t readJoystick() {
  uint8_t pressed = 0;
  for (size_t i = 0; i < Config::kInputCount; ++i) {
    if (digitalRead(Config::kJoystickPins[i]) == LOW) pressed |= 1U << i;
  }
  return pressed;
}

void scanJoystick(uint32_t now) {
  if (static_cast<uint32_t>(now - lastScan) < Config::kScanMs) return;
  lastScan = now;
  const bool changed = joystick.update(readJoystick(), now);
  if (!joystick.ready()) return;
  if (changed) {
    const JoystickColor color = joystickColor(joystick.state());
    rgbLedWrite(Config::kRgbLedPin, color.red, color.green, color.blue);
  }
  if (!announced) {
    messages.enqueue(MessageKind::Ready, now);
    messages.enqueue(MessageKind::Joystick, now, joystick.state());
    announced = true;
  } else if (changed) {
    messages.enqueue(MessageKind::Joystick, now, joystick.state());
  }
}

void transmitMessages(uint32_t now) {
  messages.prepare(joystick.state(), now);
  const int available = Serial.availableForWrite();
  if (available <= 0 || messages.remaining() == 0) return;
  size_t bytes = messages.remaining();
  if (bytes > static_cast<size_t>(available)) bytes = available;
  messages.consume(Serial.write(reinterpret_cast<const uint8_t*>(messages.data()), bytes));
}

void setup() {
  Serial.begin(Config::kBaud, SERIAL_8N1);
  for (uint8_t pin : Config::kJoystickPins) pinMode(pin, INPUT_PULLUP);
  const uint32_t now = millis();
  joystick.begin(readJoystick(), now);
  lastScan = now;
  rgbLedWrite(Config::kRgbLedPin, 0, 0, 0);
}

void loop() {
  const uint32_t now = millis();
  scanJoystick(now);
  if (announced) {
    // Bound RX work, and send snapshots through the same ordered queue as changes.
    for (size_t i = 0; i < Config::kRxBytesPerLoop && Serial.available(); ++i) {
      if (commands.feed(static_cast<char>(Serial.read()))) {
        messages.enqueue(MessageKind::Joystick, now, joystick.state());
      }
    }
    transmitMessages(now);
  }
  yield();  // GPIO scanning continues while queued serial data drains.
}
