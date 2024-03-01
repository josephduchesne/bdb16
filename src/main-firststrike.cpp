#include <Arduino.h>
#include <bdb16.h>
#include <FirstStrike.h>
#include <dshot/esc.h>
#include <functional>   // std::reference_wrapper
#include <CrsfSerial.h>
#include <FlashLog.h>

// todo: move pole pair to 2nd argument

static volatile bool alarm_fired;
volatile float left, right;

struct repeating_timer timer;

CrsfSerial radio(Serial1, CRSF_BAUDRATE);


DifferentialModel model(
        0.04445f,       // wheel diameter (m)
        0.1022f,        // wheel separation (m)
        1400.0f/4.0f,   // effective kV (kV/gear reduction)
        16.8f,          // Nominal Vbatt (V)
        5.3f,           // max velocity (m/s)
        8.47f,          // max acceleration (m/s^2)
        25.2f,          // max angular velocity (rad/s)
        111.4f          // max angular acceleration (rad/s^2)
    );

FirstStrike robot(  radio, 
                    {
                        DShot::ESC(DS1, pio0, DShot::Type::Bidir, DShot::Speed::DS600, 12, 1.0f, false), // left drive
                        DShot::ESC(DS3, pio0, DShot::Type::Bidir, DShot::Speed::DS600, 12, 1.0f, false), // right drive
                        DShot::ESC(DS2, pio0, DShot::Type::Bidir, DShot::Speed::DS600, 12, 1.0f, false)  // weapon
                    },
                    model);

unsigned long packets = 0;
void packetChannels() {packets++; }

bool repeating_timer_callback(struct repeating_timer *t) {
    // Serial2.printf("Repeat at %lld\n", time_us_64());
    robot.output();
    // todo: swap for output?
    
    return true;
}

void setup() {
    BDB16::setup(robot);

    robot.init();
    FlashLog::Setup();

    radio.onPacketChannels = &packetChannels;
    Serial1.setFIFOSize(64);
    Serial1.begin(CRSF_BAUDRATE, SERIAL_8N1);
    // Negative delay so means we will call repeating_timer_callback, and call it again
    // 500ms later regardless of how long the callback took to execute
    
    // todo: move into ESC class?
    add_repeating_timer_ms(-2, repeating_timer_callback, NULL, &timer);
}


// the loop routine runs over and over again forever:
void loop() {
    static uint64_t last_print = 0;
    
    //Serial2.printf("PC0: %d, PC1: %d, isr cnt %d %d\n", pio_sm_get_pc(pio0, 0), pio_sm_get_pc(pio0, 1), isr_count0, isr_count1);

    delay(5);
    robot.update();

    // update FlashLog
    FlashLog::WriteBasic(BDB16::read_voltage_mV(), radio.isLinkUp(), 
        {(uint16_t)radio.getChannel(1), (uint16_t)radio.getChannel(2), (uint16_t)radio.getChannel(3),
        (uint16_t)radio.getChannel(4), (uint16_t)radio.getChannel(5), (uint16_t)radio.getChannel(6)},
        {robot.escs_[0].output, robot.escs_[1].output, robot.escs_[2].output, 0}
        );
    for(auto& esc : robot.escs_) {
        DShot::Telemetry& telemetry = esc.telemetry;
        FlashLog::WriteESC(micros(), esc.pio_sm, telemetry.rpm, telemetry.temperature_C, telemetry.volts_cV, telemetry.amps_A);
    }

    // Todo: Set ESC on timer
    // Todo: Capture last telemetry time / time out telemetry validity

    // if (millis()-last_print >= 250) {
    //     last_print = millis();
    //     for(auto& esc : robot.escs_) {
    //         DShot::Telemetry& telemetry = esc.telemetry;
    //         // Serial2.printf("%d: %drpm, %dC, %02d.%02dV, %dA %0.3f\t", esc.get().pio_sm, telemetry.rpm, telemetry.temperature_C, 
    //         //                 telemetry.volts_cV/100, telemetry.volts_cV%100, telemetry.amps_A, 
    //         //                 (float)telemetry.errors*100.0f/telemetry.reads);
    //     }
    //     //Serial2.println(crsf.isLinkUp());
    // }
}
