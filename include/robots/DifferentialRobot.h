#pragma once

#include <stdint.h>
#include <array>
#include "Arduino.h"

#include "Robot.h"

#define OUTPUT_MAX_RANGE 1.0f
#define INPUT_MAX_RANGE 500.0f

class DifferentialRobot : public Robot
{
public:
    DifferentialRobot(CrsfSerial radio, const ChannelArray escs);

    void init();    // called once
    void update();  // called each loop
    void output();  // called once per output period

private:
    void ArcadeToDifferential(int drive, int steer, float &left, float &right);

};