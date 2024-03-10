#include <Arduino.h>
#include <SPI.h>
#include <bdb16.h>

#include <FastLED.h>
#define NUM_LEDS 8

CRGB leds[NUM_LEDS];

uint16_t min_val = 6680;
uint16_t max_val = 12350;

#define ENCODER_CS PIN_SPI0_CS0
void setup() {
    BDB16::init();

    SPI1.begin();
    pinMode(ENCODER_CS, OUTPUT);
    digitalWrite(ENCODER_CS, HIGH);

    FastLED.addLeds<WS2811, PIN_RGB, GRB>(leds, NUM_LEDS);
}


int32_t encoderValue() {
    static int32_t encoderZeroCrossings = 0;
    static int32_t encoderLastValue = -1;

    digitalWrite(ENCODER_CS, LOW);
    SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    uint16_t result = SPI1.transfer16(0);
    //result = (( result ) & (0x3FFF));  // ignore MSB and MSB-1 (parity and 0)
    // todo: Check parity bits?
    SPI1.endTransaction();
    digitalWrite(ENCODER_CS, HIGH);
    //Serial2.printf("Encoder Value 0x%04x\n", result & 0x3FFF);
    result = result & 0x3FFF; //result;

    // account for zero crossings
    
    int32_t delta;
    if (encoderLastValue != -1) {  // account for initialization
        delta = encoderLastValue-(int32_t)result;
        // ~180 degrees in one delta, indicates zero crossing
        if (delta>8000) encoderZeroCrossings++;
        if (delta<-8000) encoderZeroCrossings--;
    } else {
        delta = -(int32_t)result; 

    encoderLastValue = (int32_t) result;
    // SEGGER_RTT_printf(0, "Encoder Value %d delta %d zc %d final %d deg %d\n", result & 0x3FFF, delta, encoderZeroCrossings, encoderLastValue + 16384 * encoderZeroCrossings, encoderAngle(encoderLastValue + 16384 * encoderZeroCrossings));
    return encoderLastValue + 16384 * encoderZeroCrossings;
}


void loop() {
    int32_t encoder_value = encoderValue();
    int32_t leds_to_light=constrain(map(encoder_value, min_val, max_val, 0, NUM_LEDS), 0, NUM_LEDS);

    Serial2.printf("Val: %d, leds to light: %d\n", encoder_value, leds_to_light);



    for(int i=0;i<NUM_LEDS;i++) leds[7-i] = CHSV( 0, 255, (255*(i<leds_to_light)));
    FastLED.show(); // display this frame
    delay(10);
}