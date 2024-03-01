#pragma once

#include <stdint.h>
#include <array>
#include "Arduino.h"
#include <dynamics/DifferentialModel.h>

#include "Robot.h"

#define OUTPUT_MAX_RANGE 1.0f
#define INPUT_MAX_RANGE 500.0f

class DifferentialRobot : public Robot
{
public:
    DifferentialRobot(CrsfSerial& radio, const ChannelArray escs, DifferentialModel& dm);

    void init();    // called once
    void update();  // called each loop
    void output();  // called once per output period

    float left_ = 0;
    float right_ = 0;
    float left_v_ = 0;
    float right_v_ = 0;
    float v_ = 0;
    float w_ = 0;
    float last_v_ = 0;
    float last_w_ = 0;
    float last_left_ = 0;
    float last_right_ = 0;

    DifferentialModel& model_;

    uint64_t last_update_;

private:
    void ArcadeToDifferential(int drive, int steer, float &left, float &right);

};