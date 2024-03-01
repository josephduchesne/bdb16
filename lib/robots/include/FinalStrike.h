#pragma once

#include <stdint.h>
#include <array>
#include "Arduino.h"
#include <FastLED.h>
// #include <PacketSerial.h>

#include "FirstStrike.h"

class FinalStrike : public FirstStrike
{
public:
    FinalStrike(CrsfSerial& radio, const ChannelArray escs, DifferentialModel& dm);

    virtual const char* name() override { return "Final Strike"; };

    void init();
    // void update();
    // void output();

private:

};
