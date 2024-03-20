#include "FinalStrike.h"

FinalStrike::FinalStrike(CrsfSerial& radio, const ChannelArray escs, DifferentialModel& dm, float spinup_time) : FirstStrike(radio, escs, dm, spinup_time) {}

void FinalStrike::init() {
    DifferentialRobot::init();
    FastLED.addLeds<WS2811, PIN_RGB, RGB>(leds_, num_leds_).setCorrection( TypicalLEDStrip );
}