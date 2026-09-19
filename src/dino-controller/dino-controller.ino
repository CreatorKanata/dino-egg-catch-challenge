// src/dino-controller/dino-controller.ino: Verify joystick, rotary steps and shaft button via USB.
#include <Arduino.h>
#include "config.h"
#include "joystick.h"
#include "joystick_led.h"
#include "encoder.h"
#include "encoder_button.h"
#include "encoder_capture.h"
#include "serial_protocol.h"

JoystickInputs joystick;
RotaryEncoder encoder;
EncoderButton encoderButton;
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

void enqueueState(uint32_t now) {
  messages.enqueue(MessageKind::State, now, joystick.state(), encoderButton.pressed(),
                   0, EncoderCapture::readAB());
}

void scanJoystick(uint32_t now) {
  if (static_cast<uint32_t>(now - lastScan) < Config::kScanMs) return;
  lastScan = now;
  const bool changed = joystick.update(readJoystick(), now);
  const bool buttonChanged = encoderButton.update(digitalRead(Config::kEncoderSwPin) == LOW, now);
  if (joystick.ready() && changed) {
    const JoystickColor color = joystickColor(joystick.state());
    rgbLedWrite(Config::kRgbLedPin, color.red, color.green, color.blue);
  }
  if (!joystick.ready() || !encoderButton.ready()) return;
  if (!announced) {
    // Ignore rotation during startup debounce; begin from a fresh phase, without a step.
    encoder.begin(EncoderCapture::reset());
    messages.enqueue(MessageKind::Ready, now);
    enqueueState(now);
    announced = true;
  } else {
    if (changed) messages.enqueue(MessageKind::Joystick, now, joystick.state());
    if (buttonChanged) messages.enqueue(MessageKind::Button, now, 0, encoderButton.pressed());
  }
}

void scanEncoder() {
  if (!announced) return;
  // Bounded drain keeps scanning responsive even if electrical noise fills the queue.
  for (size_t i = 0; i < Config::kEncoderEdgeCapacity; ++i) {
    EncoderEdge edge{};
    bool lost = false;
    const bool available = EncoderCapture::pop(edge, lost);
    if (lost) {
      encoder.begin(edge.ab);
      messages.markOverflow();
    }
    if (!available) break;
    const int8_t delta = encoder.update(edge.ab);
    if (delta) messages.enqueue(MessageKind::Encoder, edge.ms, 0, false, delta);
  }
}

void transmitMessages(uint32_t now) {
  messages.prepare(joystick.state(), now, encoderButton.pressed(), EncoderCapture::readAB());
  const int available = Serial.availableForWrite();
  if (available <= 0 || messages.remaining() == 0) return;
  size_t bytes = messages.remaining();
  if (bytes > static_cast<size_t>(available)) bytes = available;
  messages.consume(Serial.write(reinterpret_cast<const uint8_t*>(messages.data()), bytes));
}

void setup() {
  Serial.begin(Config::kBaud, SERIAL_8N1);
  for (uint8_t pin : Config::kJoystickPins) pinMode(pin, INPUT_PULLUP);
  pinMode(Config::kEncoderClkPin, INPUT_PULLUP);
  pinMode(Config::kEncoderDtPin, INPUT_PULLUP);
  pinMode(Config::kEncoderSwPin, INPUT_PULLUP);
  const uint32_t now = millis();
  joystick.begin(readJoystick(), now);
  encoderButton.begin(digitalRead(Config::kEncoderSwPin) == LOW, now);
  attachInterrupt(Config::kEncoderClkPin, EncoderCapture::onChange, CHANGE);
  attachInterrupt(Config::kEncoderDtPin, EncoderCapture::onChange, CHANGE);
  encoder.begin(EncoderCapture::reset());
  lastScan = now;
  rgbLedWrite(Config::kRgbLedPin, 0, 0, 0);
}

void loop() {
  scanEncoder();
  const uint32_t now = millis();
  scanJoystick(now);
  if (announced) {
    // Bound RX work, and send snapshots through the same ordered queue as changes.
    for (size_t i = 0; i < Config::kRxBytesPerLoop && Serial.available(); ++i) {
      if (commands.feed(static_cast<char>(Serial.read()))) {
        enqueueState(now);
      }
    }
    transmitMessages(now);
  }
  yield();  // GPIO scanning continues while queued serial data drains.
}
