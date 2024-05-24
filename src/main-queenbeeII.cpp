#include <Arduino.h>
#include <bdb16.h>
#include <QueenBeeII.h>
#include <dshot/esc.h>
#include <functional>   // std::reference_wrapper
#include <CrsfSerial.h>

static volatile bool alarm_fired;
volatile float left, right;

struct repeating_timer timer;

CrsfSerial radio(Serial1, CRSF_BAUDRATE);

constexpr uint64_t print_interval   = 250;  // 250ms interval, 4Hz
constexpr uint64_t update_interval  = 10;   // 10ms interval, 100Hz
constexpr uint64_t log_interval     = 20;   // 20ms interval, 50Hz

QueenBeeII robot(  radio,
                    {
                        DShot::ESC(DS1, pio0, DShot::Type::Normal, DShot::Speed::DS600, 14, 0.5f, true),  // left drive
                        DShot::ESC(DS0, pio0, DShot::Type::Normal, DShot::Speed::DS600, 14, 0.5f, true), // right drive
                    }
                    );

unsigned long packets = 0;
void packetChannels() {packets++; }

bool __isr repeating_timer_callback(struct repeating_timer *t) {
    // Serial2.printf("Repeat at %lld\n", time_us_64());
    robot.output();
    
    return true;
}

void setup() {
    BDB16::init(robot);
    // FlashLog::Setup();
    robot.init();

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
    static uint64_t last_update = 0;
    static uint64_t last_log = 0;
    
    //Serial2.printf("PC0: %d, PC1: %d, isr cnt %d %d\n", pio_sm_get_pc(pio0, 0), pio_sm_get_pc(pio0, 1), isr_count0, isr_count1);

    uint64_t now = millis();
 
    if (now-last_update > update_interval) {
        if (last_update == 0) last_update = now;
        else last_update += update_interval;
        robot.update();
    }

    if (now-last_print > print_interval) {
        if (last_print == 0) last_print = now;
        else last_print += print_interval;

        // for(auto& esc : robot.escs_) {
        //     DShot::Telemetry& telemetry = esc.telemetry;
        //     Serial2.printf("%d: %drpm, %dC, %02d.%02dV, %dA %0.3f\t", esc.pio_sm, telemetry.rpm, telemetry.temperature_C, 
        //                     telemetry.volts_cV/100, telemetry.volts_cV%100, telemetry.amps_A, 
        //                     (float)telemetry.errors*100.0f/telemetry.reads);
        // }
        Serial2.printf("%d %.2f %.2f %0.2f %0.2f", robot.encoder_left_.get(), robot.encoder_left_.percent(), robot.left_, robot.left_leg_.pid_target_, robot.left_leg_.motor_out_);
        //Serial2.printf("%d %.2f ", robot.encoder_right_.get(), robot.encoder_right_.percent());
        //Serial2.printf("%d %.2f ", robot.encoder_weapon_.get(), robot.encoder_weapon_.percent());
        Serial2.println(radio.isLinkUp());
    }
}
