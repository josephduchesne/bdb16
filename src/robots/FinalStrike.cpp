#include "robots/FinalStrike.h"

FinalStrike::FinalStrike(CrsfSerial radio, const ChannelArray escs) : FirstStrike(radio, escs) {}

void FinalStrike::init() {
    DifferentialRobot::init();
    FastLED.addLeds<WS2811, PIN_RGB, RGB>(leds_, num_leds_).setCorrection( TypicalLEDStrip );
}