#include <Arduino.h>

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(900);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  Serial.write("Hello World!\n");
}
