#include "QueenBeeII.h"

QueenBeeII::QueenBeeII(CrsfSerial& radio, const ChannelArray escs) 
    : Robot(radio, escs),
      encoder_left_(PIN_SPI0_CS1, 9298, 4353),
      encoder_right_(PIN_SPI0_CS0, 5802, 10758),
      encoder_weapon_(PIN_SPI0_CS2, 4375, 96650),
      left_leg_(encoder_left_, DS3, 1500, 2000),
      right_leg_(encoder_right_, DS2, 2000, 1500)
      {}

void QueenBeeII::init() {
    Robot::init();
    last_update_ = micros();

    FastLED.addLeds<WS2811, PIN_RGB, GRB>(leds_, num_leds_).setCorrection( TypicalLEDStrip );

    left_leg_.init();
    right_leg_.init();
    encoder_weapon_.init();

    // set weapon encoder CS high
    
}


void QueenBeeII::LEDs() {
    // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
    uint8_t BeatsPerMinute = 62;
    uint8_t gHue = 0;
    CRGBPalette16 palette = PartyColors_p;
    uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
    for( int i = 0; i < num_leds_/2+1; i++) { //9948
        leds_[num_leds_-i] = leds_[i] = ColorFromPalette(palette, gHue+(i), beat+gHue-(i*35));
    }
    FastLED.show(); // display this frame
}

void QueenBeeII::update() {

    Robot::update();
    dt_ = (micros()-last_update_)/1000000.0f;  // us->s
    last_update_ = micros();

    // DShot::ESC::processTelemetryQueue();

    if (radio_.isLinkUp()) {

        // handle the arcade steering on channels 1 and 2
        int ch1_in, ch2_in;
        ch1_in = radio_.getChannel(1);
        ch2_in = radio_.getChannel(2);
        if (ch1_in > 1485 && ch1_in < 1515) ch1_in = 1500;  // Add a small deadband
        if (ch2_in > 1485 && ch2_in < 1515) ch2_in = 1500;  // Add a small deadband
        left_ = (float)(ch2_in-1500)/500.0f;
        right_ = (float)(ch1_in-1500)/500.0f;

    } else {  // radio is down
        left_ = 0.0f;
        right_ = 0.0f;
    }

    // save old values (for acceleration calculations)
    last_left_ = left_;
    last_right_ = right_;

    left_leg_.update(left_);
    right_leg_.update(right_);

    LEDs();

}

void QueenBeeII::output() {
    Robot::output();

    // encoders done on fast timer
    encoder_left_.update();
    encoder_right_.update();
    encoder_weapon_.update();

    // pid calculation done on fast timer too
    left_leg_.update(left_, !radio_.isLinkUp());
    right_leg_.update(right_, !radio_.isLinkUp());

    // left_leg_.raiseLeg((millis()/1000) % 3 == 0);
    // right_leg_.raiseLeg((millis()/1000) % 3 == 0);

    left_leg_.output();
    right_leg_.output();

    if (millis() < 3000) {
        for(auto& esc : escs_) {
            if (millis()< 2500) esc.setCommand(0);  // 1046 is the example command
            esc.setCommand(13);  // extended telemetry enable
        }
    } else {
        escs_[0].setThrottle3D(left_leg_.motor_out_);
        escs_[1].setThrottle3D(right_leg_.motor_out_);
    }
}