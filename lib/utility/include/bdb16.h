#pragma once

#include <Arduino.h>
#include <Robot.h>

namespace BDB16 {

    void init() {
        Serial2.begin(2000000);
        delay(500); // slight pause to ensure Serial2 is connected PC side
        Serial2.printf("\u001b[33mBDB16\u001b[0m Online!\n");

        analogReadResolution(12);
        pinMode(PIN_VSENSE, INPUT);
    }

    void init(Robot& robot) {
        Serial2.begin(2000000);
        delay(500); // slight pause to ensure Serial2 is connected PC side
        Serial2.printf("\u001b[33m%s\u001b[0m Online!\n", robot.name());

        analogReadResolution(12);
        pinMode(PIN_VSENSE, INPUT);
    }

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
}
