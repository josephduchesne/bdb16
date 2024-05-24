#include <Arduino.h>
#include <107-Arduino-Servo-RP2040.h>
#include <bdb16.h>

static _107_::Servo servo_left;

void setup() {
  BDB16::init();

  pinMode(LED_BUILTIN, OUTPUT);

  servo_left.attach(DS2);
  servo_left.writeMicroseconds(1000);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  servo_left.writeMicroseconds(2000);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  servo_left.writeMicroseconds(1500);
  delay(500);
}
