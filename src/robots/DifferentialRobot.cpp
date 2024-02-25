#include "robots/DifferentialRobot.h"

DifferentialRobot::DifferentialRobot(CrsfSerial radio, const ChannelArray escs, DifferentialModel& dm) 
    : Robot(radio, escs), model_(dm) {}

void DifferentialRobot::init() {
    Robot::init();
    last_update_ = micros();
}

void DifferentialRobot::update() {

    Robot::update();
    float dt = (micros()-last_update_)/1000000.0f;  // us->s
    last_update_ = micros();

    DShot::ESC::processTelemetryQueue();

    if (radio_.isLinkUp()) {

        // handle the arcade steering on channels 1 and 2
        int ch1_in, ch2_in;
        ch1_in = radio_.getChannel(1);
        ch2_in = radio_.getChannel(2);
        if (ch1_in > 1485 && ch1_in < 1515) ch1_in = 1500;  // Add a small deadband
        if (ch2_in > 1485 && ch2_in < 1515) ch2_in = 1500;  // Add a small deadband

        
        float stick_in_v = (float)(ch1_in - 1500) / 500.0f;
        stick_in_v = constrain(stick_in_v, -1.0f, 1.0f);
        
        float stick_in_w = (float)(ch2_in - 1500) / 500.0f;
        stick_in_w = constrain(stick_in_w, -1.0f, 1.0f);

        float in_v, in_w;
        // scale the sticks from [-1.0 to 1.0] to [-max to max] v and w
        model_.scaleArcadeInput(stick_in_v, stick_in_w, in_v, in_w);

        // apply model's velocity and acceleration constraints
        model_.applyConstraints(last_v_, last_w_, in_v, in_w, dt, v_, w_);

        // convert from twist v/w to L/R channels
        model_.twistToDifferentialVelocity(v_, w_, left_v_, right_v_);

        // Convert from velocity to throttle values
        left_ = model_.velocityToEscThrottle(left_v_);
        right_ = model_.velocityToEscThrottle(right_v_);

        // old: just go directly from arcade sticks to differential throttle
        //ArcadeToDifferential(ch1_in, ch2_in, left_, right_);

    } else {  // radio is down
        left_ = 0.0f;
        right_ = 0.0f;
        left_v_ = 0.0f;
        right_v_ = 0.0f;
        v_ = 0.0f;
        w_ = 0.0f;
    }

    // save old values (for acceleration calculations)
    last_v_ = v_;
    last_w_ = w_;
    last_left_ = left_;
    last_right_ = right_;

}

void DifferentialRobot::output() {
    Robot::output();

    if (millis() < 3000) {
        for(auto& esc : escs_) {
            if (millis()< 2500) esc.setCommand(0);  // 1046 is the example command
            esc.setCommand(13);  // extended telemetry enable
        }
    } else {
        escs_[0].setThrottle3D(left_);
        escs_[1].setThrottle3D(right_);
    }
}

void DifferentialRobot::ArcadeToDifferential(int drive, int steer, float &left, float &right) {
    if(drive>0) drive -= 1500;
    if(steer>0) steer -= 1500;

    // Channel mixing math from http://home.kendra.com/mauser/joystick.html
    // Get X and Y from the Joystick, do whatever scaling and calibrating you need to do based on your hardware.
    float x = (float)steer;
    float y = (float)drive;

    float v = (INPUT_MAX_RANGE - abs(x)) * (y / INPUT_MAX_RANGE) + y;
    float w = (INPUT_MAX_RANGE - abs(y)) * (x / INPUT_MAX_RANGE) + x;

    right = constrain((float)(((v + w) / 2.0)*OUTPUT_MAX_RANGE/INPUT_MAX_RANGE), -OUTPUT_MAX_RANGE, OUTPUT_MAX_RANGE);
    left = constrain((float)(((v - w) / 2.0)*OUTPUT_MAX_RANGE/INPUT_MAX_RANGE), -OUTPUT_MAX_RANGE, OUTPUT_MAX_RANGE);;
}
