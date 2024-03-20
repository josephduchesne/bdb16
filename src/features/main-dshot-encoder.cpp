#include <Arduino.h>
#include <dshot/esc.h>
#include <functional>   // std::reference_wrapper
#include <SPI.h>
#include <bdb16.h>
#include <dynamics/Constraints.h>

// todo: move pole pair to 2nd argument
DShot::ESC dshotR(DS0, pio0, DShot::Type::Bidir, DShot::Speed::DS600, 14, 0.25, false);

#define ENCODER_CS PIN_SPI0_CS0

void setup() {
    BDB16::init();
    SPI1.begin();
    pinMode(ENCODER_CS, OUTPUT);
    digitalWrite(ENCODER_CS, HIGH);

    if (!dshotR.init()) {
        Serial2.println("ESC init error :/");
    } else {
        Serial2.printf("ESC init success on SM %d :)\n", dshotR.pio_sm);
    }
}

enum class LegYawState { forward, stopForward, reverse, stopReverse, stop };
uint64_t stop_time = 50; //ms
LegYawState leg_state = LegYawState::forward;

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
    }

    encoderLastValue = (int32_t) result;
    // SEGGER_RTT_printf(0, "Encoder Value %d delta %d zc %d final %d deg %d\n", result & 0x3FFF, delta, encoderZeroCrossings, encoderLastValue + 16384 * encoderZeroCrossings, encoderAngle(encoderLastValue + 16384 * encoderZeroCrossings));
    return encoderLastValue + 16384 * encoderZeroCrossings;
}

// the loop routine runs over and over again forever:
void loop() {
    static uint64_t last_print = 0;
    static uint64_t state_time = 0;
    float throttle = 0;
    static float old_capped_throttle = 0.0f;
    static uint64_t last_send = micros();
    
    //Serial2.printf("PC0: %d, PC1: %d, isr cnt %d %d\n", pio_sm_get_pc(pio0, 0), pio_sm_get_pc(pio0, 1), isr_count0, isr_count1);

    delay(1);

    int32_t encoder_value = encoderValue();

    if (millis()< 2000) dshotR.setCommand(0);  // 1046 is the example command
    else if (millis() < 3600) dshotR.setCommand(13);  // extended telemetry enable
    else {
        switch(leg_state) {
            case LegYawState::forward:
                throttle = 1.0f;
                
                if (encoder_value>9400) {
                    leg_state = LegYawState::stopForward;
                    state_time = millis();
                }
                break;
            case LegYawState::stopForward:
                throttle = 0.0f;
                if (millis()-state_time > stop_time) {
                    leg_state = LegYawState::reverse;
                    state_time = millis();
                }
                break;
           case LegYawState::reverse:
                throttle = -1.0f;
                if (encoder_value<8500) {
                    leg_state = LegYawState::stopReverse;
                    state_time = millis();
                }
                break;
            case LegYawState::stopReverse:
                throttle = 0.0f;
                if (millis()-state_time > stop_time) {
                    leg_state = LegYawState::forward;
                    state_time = millis();
                }
                break;
            default:
                throttle = 0.0f;
                break;
        }
        if (millis()>6600) leg_state = LegYawState::stop;

        // 8.0 throttle/s^2, at 1.0 throttle = 35% of 750kV @ 16.0V ~= max V (1.0 throttle) = 4.2 rad/s, so 8.0T/s ~= 33.6 rad/s^2
        float capped_throttle = Constraints::capAccleration(old_capped_throttle, throttle, (float)(micros()-last_send)/1000000.0f, 8.0f);

        dshotR.setThrottle3D(capped_throttle);
        last_send = micros();
        old_capped_throttle = capped_throttle;
    }
    
    delay(2);

    // decode any incoming telemetry
    DShot::ESC::processTelemetryQueue();

    // Todo: Set ESC on timer
    // Todo: Timeout ESC if not set for too long
    // Todo: Capture last telemetry time / time out telemetry validity

    // if (millis()-last_print >= 8) {
    //     last_print = millis();
    if (leg_state != LegYawState::stop ) {
        DShot::Telemetry& telemetry = dshotR.telemetry;
        Serial2.printf("%.5f %u %u %02d.%02d %0.3f %d %d %0.3f %0.3f\n", (float)millis()/1000.0f, telemetry.rpm, telemetry.temperature_C, 
                        telemetry.volts_cV/100, telemetry.volts_cV%100, 
                        (float)telemetry.errors*100.0f/telemetry.reads, encoder_value, leg_state, throttle, old_capped_throttle);
    }
}
