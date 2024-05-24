#include "mechanics/Leg2D.h"

Leg2D::Leg2D(Encoder& encoder, uint8_t servo_pin, uint16_t servo_down, uint16_t servo_up)
: 
    encoder_(encoder),
    servo_pin_(servo_pin),
    servo_down_(servo_down),
    servo_up_(servo_up),
    servo_ms_(servo_down),
    pid_(&encoder_percent_, &motor_out_, &pid_target_)
{

}

void Leg2D::init() {
    servo_.attach(servo_pin_);
    encoder_.init();

    pid_.SetTunings(Kp_, Ki_, Kd_);
    pid_.SetOutputLimits(-1.0f, 1.0f);
    pid_.SetSampleTimeUs(20000); // 50Hz
    pid_.SetMode(QuickPID::Control::manual);

    raiseLeg(false);  // leg starts lowered
}

void Leg2D::SetAction(Leg2D::Action new_action_) {
    if (action_ == new_action_) return;

    action_ = new_action_;
    action_start_ = millis();
    first_update_ = true;

    switch(action_) {
        case Leg2D::Action::raise:
        case Leg2D::Action::lower:
        case Leg2D::Action::stop:
            pid_.SetMode(QuickPID::Control::manual);
            break;
        default:
            pid_.SetMode(QuickPID::Control::automatic);
    }
}

bool Leg2D::ActionTimeout() {
    auto& max_time = MaxActionTimes_[(uint)action_];

    if (max_time == 0) return false; // no timeout

    return (millis() - action_start_) >= max_time;
}

void Leg2D::update(float throttle, bool hard_stop) {

    encoder_percent_ = encoder_.percent();
    pid_target_ = 50.0 + 49.0*throttle;

    if (hard_stop) {
        SetAction(Leg2D::Action::stop);
    }
    
    switch(action_) {
        case Leg2D::Action::pid:
            // pid takes over
            raiseLeg(false);  // leg down

            if (pid_target_ > 95.0 && encoder_percent_ > 95.0) {
                cycle_target_ = 5.0;
                SetAction(Leg2D::Action::raise);
            }

            if (pid_target_ < 5.0 && encoder_percent_ < 5.0) {
                cycle_target_ = 95.0;
                SetAction(Leg2D::Action::raise);
            }

            break;
        case Leg2D::Action::raise:
            motor_out_ = 0.0f;
            raiseLeg(true);

            if (ActionTimeout()) SetAction(Leg2D::Action::cycle);
            break;

        case Leg2D::Action::cycle:
            pid_target_ = cycle_target_;

            raiseLeg(true);  // leg remains up

            // if we ran out of time or are within 5% of target, we're done
            if (ActionTimeout() || fabs(pid_target_ - encoder_percent_) < 5.0) {
                SetAction(Leg2D::Action::lower);
            }
            break;

        case Leg2D::Action::lower:
            motor_out_ = 0.0f;
            raiseLeg(false);

            if (ActionTimeout()) SetAction(Leg2D::Action::pid);

            break;
        case Leg2D::Action::stop:  // fallthrough
            if (!hard_stop) {
                SetAction(Leg2D::Action::pid);
            }
            raiseLeg(false);
            motor_out_ = 0.0f;
            break;
        default:
            motor_out_ = 0.0f;
            break;
    }


    pid_.Compute();

    // close enough for horseshoes and handgrenades
    if (fabs(pid_target_ - encoder_percent_) < 1.0) {
        motor_out_ = 0.0f;
    }

}

void Leg2D::raiseLeg(bool up) {
    servo_ms_ = up ? servo_up_ : servo_down_;
}
