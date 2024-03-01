#pragma once

#include <stdint.h>
#include <array>
#include "Arduino.h"

#include <dshot/esc.h>
#include <CrsfSerial.h>

#ifndef OUTPUT_CHANNELS
#define OUTPUT_CHANNELS 3
#endif
#define ChannelArray std::array<DShot::ESC, OUTPUT_CHANNELS>

class Robot
{
public:
    Robot(CrsfSerial& radio, const ChannelArray escs);
    ~Robot();

    void init();    // called once
    void update();  // called each loop
    void output();  // called once per output period
    virtual const char* name() = 0;

// protected:
    ChannelArray escs_;
    CrsfSerial& radio_;
};