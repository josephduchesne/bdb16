#pragma once

#include <107-Arduino-Servo-RP2040.h>
#include <encoder.h>

class Leg2D
{
public:
    
    Leg2D(Encoder& encoder, uint8_t servo_pin, uint16_t servo_down=2000, uint16_t servo_up=2000);

    const uint16_t servo_down_;
    const uint16_t servo_up_;
    uint16_t servo_ms_;
    const uint8_t servo_pin_;
    _107_::Servo servo_;
    Encoder& encoder_;

    void raiseLeg(bool up);

    void init() {
        servo_.attach(servo_pin_);
        encoder_.init();
    }

    void update() {
        // encoder updates need to be done faster
    }

    void output() {
        servo_.writeMicroseconds(servo_ms_);
    }

private:


};
