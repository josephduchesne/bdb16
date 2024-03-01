#pragma once

#include <stdint.h>
#include <array>
#include "Arduino.h"
#include <FastLED.h>
// #include <PacketSerial.h>

#include "DifferentialRobot.h"

class FirstStrike : public DifferentialRobot
{
public:
    FirstStrike(CrsfSerial& radio, const ChannelArray escs, DifferentialModel& dm);

    void init();
    void update();
    void output();

    virtual const char* name() override { return "First Strike"; };

    void LEDs();

    volatile float weapon_ = 0.0f;
    constexpr static size_t num_leds_ = 16;
    CRGB leds_[num_leds_];

private:
    int telemetry_counter_ = 0;
    int telemetry_ratio_ = 16;
};
