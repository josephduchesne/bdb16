#include "robots/FirstStrike.h"

FirstStrike::FirstStrike(CrsfSerial radio, const ChannelArray escs) : DifferentialRobot( radio, escs ) {}

void FirstStrike::init() {
    DifferentialRobot::init();
    FastLED.addLeds<WS2811, PIN_RGB, GRB>(leds_, num_leds_).setCorrection( TypicalLEDStrip );
}

void FirstStrike::LEDs() {
    uint8_t BeatsPerMinute = 80;
    CRGBPalette16 palette = PartyColors_p;
    uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
    static uint8_t gHue = 0;

    if (radio_.isLinkUp()) { 
        for( int i = 0; i < num_leds_/2; i++) { //9948
            leds_[i] = ColorFromPalette(palette, (int8_t)((weapon_)*45.0f)+(i*2), beat+(i*10));
        }

        for( int i = num_leds_/2; i < num_leds_; i++) { //9948
            leds_[i] = ColorFromPalette(palette, (int8_t)((weapon_)*45.0f)+((num_leds_-i)*2), beat+((num_leds_-i)*10));
        }
        fadeToBlackBy( leds_, num_leds_, 50);
        leds_[3-(int)(right_*3.7-0.5f)] = CHSV(0,128,255);//ColorFromPalette(palette, 90, 255);
        leds_[8+4+(int)(left_*3.7-0.5f)] = CHSV(0,128,255);//ColorFromPalette(palette, 90, 255);

        // todo: weapon animation
    } else { // idle animation
        fadeToBlackBy( leds_, num_leds_, 20);
        leds_[beatsin16( 60, 0, num_leds_/2-1 )] |= ColorFromPalette(HeatColors_p, gHue>127 ? 255-gHue : gHue, 255, LINEARBLEND);
        leds_[num_leds_-1-beatsin16( 60, 0, num_leds_/2-1 )] |= ColorFromPalette(HeatColors_p, gHue>127 ? 255-gHue : gHue, 255, LINEARBLEND);
        gHue+=1;
    }

    FastLED.show(); // display this frame
}

void FirstStrike::update() {
    DifferentialRobot::update();

    if (radio_.isLinkUp()) { 
        int16_t weapon_in = radio_.getChannel(3);
        weapon_ = 0.0f;
        if (weapon_in <= 1485 || weapon_in >= 1515) { // Ignore a small deadband
            weapon_ = constrain(((float)(weapon_in-1500)*2.0f/1000.0f), -1.0f, 1.0f);
        }

        // SEGGER_RTT_printf(0, "ch2: %d, %d, %d\n", crsf.getChannel(3), ch2);
    } else {
        weapon_ = 0.0f;  // weapon throttle disabled when radio is disconnected
    }

    LEDs();

    // Telemetry: Todo: re-add
    // if ((telemetry_counter_++)%telemetry_ratio_ == 0) {
    //     // battery telemetry
    //     crsf_sensor_battery_t crsfbatt = { 0 };
    //     crsfbatt.voltage = 321; // htobe16(batteryVoltage()/10);  // 3.5V
    //     crsfbatt.current = htobe16(123);
    //     radio_.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfbatt, sizeof(crsfbatt));
    // }

    // esc telemetry?
    //SEGGER_RTT_printf(0, "Telemetry: %d?\n", Serial2.available());
    //print_telemetry();
}

void FirstStrike::output() {
    DifferentialRobot::output();

    if (millis() > 7000) {
        escs_[2].setThrottle3D(weapon_);
    }
}
