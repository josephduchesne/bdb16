#include <Arduino.h>


uint16_t read_voltage_mV() {
    static uint64_t era_mV = 0;

    // will be a 12 bit number (0-4095) where max input is 3.3V pr 3300mV
    uint64_t value = (uint64_t)analogRead(PIN_VSENSE) * 110u * 3300u / 10u / 4096u; 

    if (era_mV == 0) {
      era_mV = value;
    } else {
      era_mV = (era_mV * 90 + value * 10) / 100;
    }
    return (uint16_t) era_mV;
    
}

void setup() {
    Serial2.begin(115200);
    analogReadResolution(12);
    pinMode(PIN_VSENSE, INPUT);
}

void loop() {
  Serial2.println(read_voltage_mV());
  delay(10);
}