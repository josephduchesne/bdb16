#pragma once

#include <stdint.h>
#include <array>
#include "Arduino.h"
#include <FastLED.h>
// #include <PacketSerial.h>

#include "robots/FirstStrike.h"

class FinalStrike : public FirstStrike
{
public:
    FinalStrike(CrsfSerial radio, const ChannelArray escs);

    void init();
    // void update();
    // void output();

private:

};
