#pragma once

#include <107-Arduino-Servo-RP2040.h>
#include <encoder.h>
#include <QuickPID.h>

class Leg2D
{
public:


    Leg2D(Encoder& encoder, uint8_t servo_pin, uint16_t servo_down=2000, uint16_t servo_up=2000);

    enum class Action : uint8_t {stop, pid, raise, cycle, lower, MAX};  // controller mode
    const uint64_t MaxActionTimes_[(uint)Action::MAX] = {0, 0, 75, 350, 75}; // ms per controller mode before it goes next

    const uint16_t servo_down_;
    const uint16_t servo_up_;
    uint16_t servo_ms_;
    const uint8_t servo_pin_;
    _107_::Servo servo_;
    Encoder& encoder_;
    float motor_out_ = 0.0f;
    float encoder_percent_ = 50.0f;
    float pid_target_ = 0.0f;
    bool first_update_ = true;

    Action action_ = Action::stop;
    uint64_t action_start_ = 0;
    float cycle_target_ = 0.0f;

    const float Kp_ = 0.05f;
    const float Ki_ = 0.0f;
    const float Kd_ = 0;

    QuickPID pid_;

    void raiseLeg(bool up);

    void SetAction(Leg2D::Action new_action_);

    bool ActionTimeout();

    void init();

    void update(float throttle, bool hard_stop=false);

    void output() {
        servo_.writeMicroseconds(servo_ms_);
    }

private:


};
