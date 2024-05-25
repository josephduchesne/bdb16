#pragma once

#include <encoder.h>
#include <SoftwareSerial.h>
#include <VescUart.h>

class Hammer
{
public:


    Hammer(Encoder& encoder, SoftwareSerial& serial);

    enum class Action : uint8_t {locked, stop, swing, wait1, rev1, rev2, wait2, MAX};  // Hammer Action States
    enum class Mode : uint8_t {off, current, rpm, brake, MAX};  // ESC mode
    enum class HammerSwing : uint8_t {Attack, SelfRight, MAX};  // Types of hammer swing

    const Mode ActionModes_[(uint)HammerSwing::MAX][(uint)Action::MAX] = {  // mode for each Action
        {Mode::off, Mode::off, Mode::current, Mode::off, Mode::rpm, Mode::current, Mode::off},  // Attack
        {Mode::off, Mode::off, Mode::rpm, Mode::off, Mode::rpm, Mode::current, Mode::off}       // SelfRight
    };
    const uint64_t MaxActionTimes_[(uint)HammerSwing::MAX][(uint)Action::MAX] = { // timeout for each Action (ms)
        {0, 0, 120, 290, 150, 200, 100}, // ms per Attack esc mode before it goes next
        {0, 0, 350, 290, 150, 250, 100}  // ms per SelfRight esc mode before it goes next
    };
    const float ActionValues_[(uint)HammerSwing::MAX][(uint)Action::MAX] = {  // output value associated with each action
        {0, 0, -40.0f /* A */,     0.0, 1000.0f /* RPM*/, 2.0f /*A*/, 0.0},
        {0, 0, -2000.0f /* RPM */, 0.0, 1000.0f /* RPM*/, 2.0f /*A*/, 0.0},
    };

    SoftwareSerial& vesc_serial_;
    VescUart vesc_;

    Encoder& encoder_;
    float motor_out_ = 0.0f;
    float encoder_percent_ = 0.0f;
    bool first_update_ = true;

    Action action_ = Action::locked;
    HammerSwing swing_ = HammerSwing::Attack;

    uint64_t action_start_ = 0;

    void SetAction(Action new_action_);

    bool ActionTimeout();

    void OutputActionMode();

    void init();

    void update(bool swing, bool self_right, bool hard_stop=false);

    void output() {
        
    }

private:


};
