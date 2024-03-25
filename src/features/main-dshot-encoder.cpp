#include <Arduino.h>
#include <dshot/esc.h>
#include <functional>   // std::reference_wrapper
#include <SPI.h>
#include <bdb16.h>
#include <dynamics/Constraints.h>

// todo: move pole pair to 2nd argument
DShot::ESC dshotR(DS0, pio0, DShot::Type::Normal, DShot::Speed::DS300, 14, 1.0, false);

#define ENCODER_CS PIN_SPI0_CS0

int32_t min_val = -3650;
int32_t max_val = 1220;
int32_t stopping_range = 250;
int32_t encoder_phase_lag = 300;
uint64_t stop_time = 450; //ms
uint64_t hold_time = 45; // ms

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

enum class LegYawState { forward, stopForward, holdHigh, reverse, stopReverse, holdLow, stop };

LegYawState leg_state = LegYawState::forward;

int32_t encoderValue() {
    static int32_t encoderZeroCrossings = 0;
    static int32_t encoderLastValue = -1;

    digitalWrite(ENCODER_CS, LOW);
    SPI1.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));
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
        if (delta<-8000) encoderZeroCrossings--;
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
    static uint64_t last_send = 0;
    
    //Serial2.printf("PC0: %d, PC1: %d, isr cnt %d %d\n", pio_sm_get_pc(pio0, 0), pio_sm_get_pc(pio0, 1), isr_count0, isr_count1);

    delay(1);

    int32_t encoder_value = encoderValue();

    if (millis()< 2000) dshotR.setCommand(0);  // 1046 is the example command
    else if (millis() < 3600) dshotR.setCommand(13);  // extended telemetry enable
    else {
        switch(leg_state) {
            case LegYawState::forward:
                throttle = 1.0f;
                
                if (encoder_value > (max_val + min_val)/2-encoder_phase_lag ) {
                    leg_state = LegYawState::stopForward;
                    state_time = millis();
                }
                break;
            case LegYawState::stopForward:
                throttle = 0.0f;
                if ( (millis()-state_time > stop_time || encoder_value > max_val-stopping_range) && fabs(old_capped_throttle) < 0.01) {
                    leg_state = LegYawState::holdHigh;
                    state_time = millis();
                }
                break;
            case LegYawState::holdHigh:
                throttle = 0.0f;
                if ( millis()-state_time > hold_time ) {
                    leg_state = LegYawState::reverse;
                    state_time = millis();
                }
                break;
           case LegYawState::reverse:
                throttle = -1.0f;
                if (encoder_value < (max_val + min_val)/2+encoder_phase_lag  ) {
                    leg_state = LegYawState::stopReverse;
                    state_time = millis();
                }
                break;
            case LegYawState::stopReverse:
                throttle = 0.0f;
                if ( (millis()-state_time > stop_time  || encoder_value < min_val+stopping_range) && fabs(old_capped_throttle) < 0.01) {
                    leg_state = LegYawState::holdLow;
                    state_time = millis();
                }
                break;
            case LegYawState::holdLow:
                throttle = 0.0f;
                if ( millis()-state_time > hold_time ) {
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
        
        float dt = (float)(micros()-last_send)/1000000.0f;
        if (last_send == 0) dt = 0.00001f;  // fix infinite acceleration at first timestep
        float capped_throttle = Constraints::capAccleration(old_capped_throttle, throttle, dt, 16.0f);

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
    if (leg_state != LegYawState::stop && millis() <= 6600) {
        DShot::Telemetry& telemetry = dshotR.telemetry;
        Serial2.printf("%.5f %u %u %02d.%02d", (float)micros()/1000000.0f, telemetry.rpm, telemetry.temperature_C, 
                        telemetry.volts_cV/100, telemetry.volts_cV%100);
        Serial2.printf(" %u %0.3f %d %d %0.3f %0.3f %u\n", telemetry.amps_A,
                        (float)telemetry.errors*100.0f/telemetry.reads, encoder_value, leg_state, throttle, old_capped_throttle, dshotR.output);
    }
}
