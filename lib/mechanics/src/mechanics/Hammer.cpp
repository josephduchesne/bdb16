#include "mechanics/Hammer.h"

Hammer::Hammer(Encoder& encoder, SoftwareSerial& serial)
: 
    encoder_(encoder),
    vesc_serial_(serial),
    vesc_(10 /*ms timeout*/)
{

  vesc_.setSerialPort(&vesc_serial_);
//   vesc_.setDebugPort(&Serial2);
}

void Hammer::init() {
    encoder_.init();
}

void Hammer::SetAction(Hammer::Action new_action_) {
    if (action_ == new_action_) return;

    Serial2.printf("Hammer SetAction [%u][%u] at %lu\n", (uint)swing_, (uint)new_action_, millis());

    action_ = new_action_;
    action_start_ = millis();
    first_update_ = true;
}

void Hammer::OutputActionMode() {
    Hammer::Mode current_mode = Hammer::ActionModes_[(uint)swing_][(uint)action_];
    motor_out_ = Hammer::ActionValues_[(uint)swing_][(uint)action_];

    //if (current_mode != Mode::off) Serial2.printf("Hammer: %u, %.2f\n", (uint)current_mode, motor_out_);
    //return;

    switch(current_mode) {  // off, current, rpm, brake
        case Mode::off:
            vesc_.setCurrent(0.0f);  // not sure if this is the optimal way to command a stop?
            break;
        case Mode::current:
            vesc_.setCurrent(motor_out_);
            break;
        case Mode::rpm:
            vesc_.setRPM(motor_out_);
            break;
        case Mode::brake:
            vesc_.setBrakeCurrent(motor_out_);
            break;
        default:
            // assume off as well
            vesc_.setCurrent(0.0f);
            break;
    }
}

bool Hammer::ActionTimeout() {
    uint64_t max_time = MaxActionTimes_[(uint)swing_][(uint)action_];

    if (max_time == 0) return false; // no timeout

    return (millis() - action_start_) >= max_time;
}

void Hammer::update(bool swing, bool self_right, bool hard_stop) {

    encoder_percent_ = encoder_.percent();

    if (hard_stop) {
        SetAction(Hammer::Action::locked);
    }
    
    switch(action_) {  //  locked, stop, swing, wait1, rev1, rev2, wait2, MAX
        case Hammer::Action::locked:
            if (!hard_stop && !swing && !self_right) {  // unlock only if no swing enabled currently
                SetAction(Hammer::Action::stop);  // unlocked and ready to swing
            }
            break;
        case Hammer::Action::stop:
            if (swing) {  // begin an attack swing
                swing_ = HammerSwing::Attack;
                SetAction(Hammer::Action::swing);
            } else if (self_right) {  // begin a self-right swing
                swing_ = HammerSwing::SelfRight;
                SetAction(Hammer::Action::swing);
            }
            break;
        case Hammer::Action::swing:
            if (ActionTimeout() || encoder_percent_ > 90.0) {  // proceed until swing >90% travel, or time out
                SetAction(Hammer::Action::wait1);
            }
            break;
        case Hammer::Action::wait1:
            if (ActionTimeout()) {  // wait for dwell on impact
                SetAction(Hammer::Action::rev1);
            }
            break;
        case Hammer::Action::rev1:
            if (ActionTimeout() || encoder_percent_ < 50.0f) { // mostly reversed now
                SetAction(Hammer::Action::rev2);
            }
            break;
        case Hammer::Action::rev2:
            if (ActionTimeout() || encoder_percent_ < 5.0f) { // gently reverse the rest of the way
                SetAction(Hammer::Action::wait2);
            }
            break;
        case Hammer::Action::wait2:
            if (ActionTimeout()) { // wait a little bit before we can fire again
                SetAction(Hammer::Action::stop);
            }
            break;
        default:
            SetAction(Hammer::Action::locked);  // not sure how we'd get here
            break;
    }

    // write out to VESC
    OutputActionMode();

}
