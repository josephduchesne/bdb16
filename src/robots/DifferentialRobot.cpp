#include "robots/DifferentialRobot.h"

DifferentialRobot::DifferentialRobot(CrsfSerial radio, const ChannelArray escs) : Robot{ radio, escs } {}

void DifferentialRobot::init() {
    Robot::init();
}

void DifferentialRobot::update() {
    Robot::update();

    DShot::ESC::processTelemetryQueue();



        if (radio_.isLinkUp()) {

            // handle the arcade steering on channels 1 and 2
            int ch1_in, ch2_in;
            ch1_in = radio_.getChannel(1);
            ch2_in = radio_.getChannel(2);
            if (ch1_in > 1485 && ch1_in < 1515) ch1_in = 1500;  // Add a small deadband
            if (ch2_in > 1485 && ch2_in < 1515) ch2_in = 1500;  // Add a small deadband
            ArcadeToDifferential(ch1_in, ch2_in, left_, right_);

        } else {  // radio is down
            left_ = 0.0f;
            right_ = 0.0f;
        }

}

void DifferentialRobot::output() {
    Robot::output();

    if (millis() < 7000) {
        for(auto& esc : escs_) {
            if (millis()< 6000) esc.setCommand(0);  // 1046 is the example command
            else if (millis() < 7000) esc.setCommand(13);  // extended telemetry enable
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
