#pragma once

#include <stdint.h>
#include <array>
#include "Arduino.h"
#include <FastLED.h>
#include <FastLED.h>
#include <mechanics/Leg2D.h>
#include <mechanics/Hammer.h>
#include "Robot.h"
#include <encoder.h>
#include <SoftwareSerial.h>

class QueenBeeII : public Robot
{
public:
    QueenBeeII(CrsfSerial& radio, const ChannelArray escs, SoftwareSerial& serial);

    void init();
    void update();
    void output();

    virtual const char* name() override { return "Queen Bee II"; };

    void LEDs();

    constexpr static size_t num_leds_ = 25;
    CRGB leds_[num_leds_];

    float left_ = 0;
    float right_ = 0;
    bool fire_hammer_ = false;
    bool fire_self_right_ = false;

    const uint8_t weapon_cs_ = PIN_SPI0_CS0;
    const uint8_t left_encoder_cs_ = PIN_SPI0_CS1;
    const uint8_t right_encoder_cs_ = PIN_SPI0_CS2;

    Encoder encoder_left_;
    Encoder encoder_right_;
    Encoder encoder_weapon_;
    Leg2D left_leg_;
    Leg2D right_leg_;
    Hammer hammer_;

private:
    int telemetry_counter_ = 0;
    int telemetry_ratio_ = 16;


    //copied over from differential robot, maybe merge into Robot?
    uint64_t last_update_;
    float dt_ = 0.0f; // timestep of previous update step

};
