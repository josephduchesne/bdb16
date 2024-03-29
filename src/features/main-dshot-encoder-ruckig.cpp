#include <Arduino.h>
#include <dshot/esc.h>
#include <functional>   // std::reference_wrapper
#include <SPI.h>
#include <bdb16.h>
#include <ruckig/ruckig.hpp>
#include <dynamics/Constraints.h>

// todo: move pole pair to 2nd argument
DShot::ESC dshotR(DS0, pio0, DShot::Type::Normal, DShot::Speed::DS300, 14, 1.0, true);


#define ENCODER_CS PIN_SPI0_CS0

int32_t min_val = -3729;
int32_t max_val = 1441;
uint64_t hold_time = 50; // ms
uint64_t full_send_time = 65; // ms
constexpr double control_latency_s = 0.01; // control latency, estimated from graphs. Update whenever blheli32 params are changed :/

constexpr double deg_to_rad = 1.0/180*M_PI;
constexpr double rpm_to_rad_per_s = 2.0*M_PI/60.0;
constexpr double gear_ratio = 21.5*80.0/20.0; 
constexpr double full_throttle = 16.0*1500.0/gear_ratio*rpm_to_rad_per_s;


double encoderRads(int32_t val) {
    return (double)(val-(max_val+min_val)/2)/16384.0*TWO_PI;
}


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

using namespace ruckig;
constexpr double loop_time = 1.0/280.0;
constexpr long int loop_time_us = (long int)(loop_time*1e6);
Ruckig<1> otg(loop_time);  // 280Hz control cycle
InputParameter<1> input;
OutputParameter<1> output;

// hold for a max of 100ms (in 280Hz loops)
constexpr int max_hold_loops = 100 * 1000/loop_time_us;
int hold_loops = 0;

enum class LegYawState { ruckigStartFW, ruckigRunFW, holdHigh, ruckigStartREV, ruckigRunREV, holdLow, stop };

LegYawState leg_state = LegYawState::ruckigStartFW;

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

    // Set input parameters
    input.current_position = {55*deg_to_rad};
    input.current_velocity = {0.0};
    input.current_acceleration = {0.0};

    input.target_position = {-55.0*deg_to_rad};
    input.target_velocity = {0.0};
    input.target_acceleration = {0.0};

    input.max_velocity = {full_throttle};
    input.max_acceleration = {800.0};
    input.max_jerk = {20000.0};

    // Set minimum duration (equals the trajectory duration when target velocity and acceleration are zero)
    input.minimum_duration = 0.1;

    // set initial direction based on encoder position
    if (encoderRads(encoderValue())>0.0) {
        leg_state = LegYawState::ruckigStartREV;
    } else {
        leg_state = LegYawState::ruckigStartFW;
    }
}




// the loop routine runs over and over again forever:
void loop() {
    static uint64_t last_print = 0;
    static uint64_t state_time = 0;
    float throttle = 0;
    static float old_capped_throttle = 0.0f;
    static uint64_t last_send = 0;
    
    //Serial2.printf("PC0: %d, PC1: %d, isr cnt %d %d\n", pio_sm_get_pc(pio0, 0), pio_sm_get_pc(pio0, 1), isr_count0, isr_count1);

    long int start = micros();

    int32_t encoder_value = encoderValue();

    if (millis()< 2000) dshotR.setCommand(0);  // 1046 is the example command
    else if (millis() < 3600) dshotR.setCommand(13);  // extended telemetry enable
    else {
        switch(leg_state) {
            case LegYawState::ruckigStartFW:
                otg.reset();

                input.current_position = {encoderRads(encoder_value)};
                //printf("init ruckig fw at %.5f\n", input.current_position[0]);
                input.current_velocity = {0.0};
                input.current_acceleration = {0.0};

                input.target_position = {(55-6)*deg_to_rad};
                input.target_velocity = {0.0};
                input.target_acceleration = {0.0};

                // short circuit acceleration phase, since the blheli32 sucks at that and can handle the acceleration
                // phase better if we just give it the target velocity directly
                for (int i=0; i<max_hold_loops;i++) {
                    hold_loops = i;

                     if (otg.update(input, output) == Result::Working) {
                        output.pass_to_input(input);  // pure open loop
                        if (output.new_acceleration[0] == 0.0) break;
                     } else {
                        //todo: handle this case
                        printf("Hold loop finished trajectory?\n");
                     }
                }

                //printf("hold loops %d , result velocity = %f\n", hold_loops, output.new_velocity[0]/full_throttle);
                if (hold_loops == 0) {
                    printf("No hold loops?\n");
                }
                
                //todo: combine fw and rev states?
                
                leg_state = LegYawState::ruckigRunFW;
                state_time = millis();
                // Fallthrough
            case LegYawState::ruckigRunFW:

                if (hold_loops>0) {
                    hold_loops--;
                    throttle = output.new_velocity[0]/full_throttle;
                    if (hold_loops == 0) {
                        //printf("After last hold loop: position: %f vs expected %f ad %f rad/s\n", encoderRads(encoder_value), output.new_position[0], output.new_velocity[0]);
                        // wait until the position + vel*latency is past the original position for this control loop before continuing
                        if (encoderRads(encoder_value) + control_latency_s*output.new_velocity[0] < output.new_position[0]) {
                            hold_loops = 1;
                        }

                        //maybe re-init at velocity?
                    }
                } else {

                    //printf("Error: %f\n", fabs(input.current_position[0]-input.target_position[0]) );
                    if (otg.update(input, output) == Result::Working) {
                        throttle = output.new_velocity[0]/full_throttle;
                        
                        //printf("acceleration: %f\n", output.new_acceleration);
                        //output.new_position = {encoderRads(encoder_value)};  // plus velocity?
                        output.pass_to_input(input);  // pure open loop
                        
                    } else {
                        throttle = 0.0;
                        leg_state = LegYawState::holdHigh;
                        state_time = millis();
                    }
                }

                if (millis() - state_time < full_send_time && throttle != 0.0) {
                    throttle = 1.0;
                }

                break;
            case LegYawState::holdHigh:
                throttle = 0.0f;
                if ( millis()-state_time > hold_time ) {
                    leg_state = LegYawState::ruckigStartREV;
                    state_time = millis();
                }
                break;
           case LegYawState::ruckigStartREV:
                otg.reset();

                input.current_position = {encoderRads(encoder_value)};
                //printf("init ruckig rev at %.5f\n", input.current_position[0]);
                input.current_velocity = {0.0};
                input.current_acceleration = {0.0};

                input.target_position = {-(55-6)*deg_to_rad};
                input.target_velocity = {0.0};
                input.target_acceleration = {0.0};
                

                // short circuit acceleration phase, since the blheli32 sucks at that and can handle the acceleration
                // phase better if we just give it the target velocity directly
                for (int i=0; i<max_hold_loops;i++) {
                    hold_loops = i;

                     if (otg.update(input, output) == Result::Working) {
                        output.pass_to_input(input);  // pure open loop
                        if (output.new_acceleration[0] == 0.0) break;
                     } else {
                        //todo: handle this case
                        printf("Hold loop finished trajectory?\n");
                     }
                }

                //printf("hold loops %d , result velocity = %f\n", hold_loops, output.new_velocity[0]/full_throttle);
                if (hold_loops == 0) {
                    printf("No hold loops?\n");
                }

                leg_state = LegYawState::ruckigRunREV;
                state_time = millis();
                // Fallthrough
            case LegYawState::ruckigRunREV:
                throttle = 0.0f;

                if (hold_loops>0) {
                    hold_loops--;
                    throttle = output.new_velocity[0]/full_throttle;

                    if (hold_loops == 0) {
                        //printf("After last hold loop: position: %f vs expected %f ad %f rad/s\n", encoderRads(encoder_value), output.new_position[0], output.new_velocity[0]);
                        // wait until the position + vel*latency is past the original position for this control loop before continuing
                        if (encoderRads(encoder_value) + control_latency_s*output.new_velocity[0] > output.new_position[0]) {
                            hold_loops = 1;
                        }

                        //maybe re-init at velocity?
                    }
                } else {

                    //input.current_position = {encoderRads(encoder_value)};
                    if (otg.update(input, output) == Result::Working) {
                        throttle = output.new_velocity[0]/full_throttle;
                        output.pass_to_input(input);  // pure open loop
                    } else {
                        throttle = 0.0;
                        leg_state = LegYawState::holdLow;
                        state_time = millis();
                    }
                }

                if (millis() - state_time < full_send_time && throttle != 0.0) {
                    throttle = -1.0;
                }

                break;
            case LegYawState::holdLow:
                throttle = 0.0f;
                if ( millis()-state_time > hold_time ) {
                    leg_state = LegYawState::ruckigStartFW;
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
        //float capped_throttle = Constraints::capAccleration(old_capped_throttle, throttle, dt, 16.0f);
        float capped_throttle = throttle; // ruckig does its own cap

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

    //wait out the remainder of our loop time
    long int remaining_time = (long int)loop_time_us-(long int)(micros()-start);
    if (remaining_time>0) delayMicroseconds(remaining_time);
}
