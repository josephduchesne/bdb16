#pragma once

#include "robots/Robot.h"

Robot::Robot(CrsfSerial radio, const ChannelArray escs): radio_(radio), escs_{ escs } {

    // init dshot output
    // TODO
}

Robot::~Robot() {
}

void Robot::init() {
    for(auto& esc : escs_) {
        esc.init();
    }
}

void Robot::update() {
    radio_.loop();
}

void Robot::output() {
    
}
