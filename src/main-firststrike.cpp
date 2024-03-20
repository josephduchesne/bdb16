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

constexpr uint64_t print_interval   = 250;  // 250ms interval, 4Hz
constexpr uint64_t update_interval  = 10;   // 10ms interval, 100Hz
constexpr uint64_t log_interval     = 20;   // 20ms interval, 50Hz

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
                    model,
                    0.8f   // 0.8 second spin up time acceleration cap for drum
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
    FlashLog::Setup();
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
        last_update = now;
        robot.update();
    }

    // Todo: Set ESC on timer
    // Todo: Timeout ESC if not set for too long
    // Todo: Capture last telemetry time / time out telemetry validity

    // update FlashLog
    if (now-last_log > log_interval) {
        last_log = now;
        FlashLog::WriteBasic(BDB16::read_voltage_mV(), radio.isLinkUp(), 
            {(uint16_t)radio.getChannel(1), (uint16_t)radio.getChannel(2), (uint16_t)radio.getChannel(3),
            (uint16_t)radio.getChannel(4), (uint16_t)radio.getChannel(5), (uint16_t)radio.getChannel(6)},
            {robot.escs_[0].output, robot.escs_[1].output, robot.escs_[2].output, 0}
            );
        for(auto& esc : robot.escs_) {
            DShot::Telemetry& telemetry = esc.telemetry;
            FlashLog::WriteESC(micros(), esc.pio_sm, telemetry.rpm, telemetry.temperature_C, telemetry.volts_cV, telemetry.amps_A,
                               telemetry.debug1, telemetry.debug2, telemetry.stress, telemetry.status);
        }
    }

    if (now-last_print > print_interval) {
        last_print = now;

        for(auto& esc : robot.escs_) {
            DShot::Telemetry& telemetry = esc.telemetry;
            Serial2.printf("%d: %drpm, %dC, %02d.%02dV, %dA %0.3f\t", esc.pio_sm, telemetry.rpm, telemetry.temperature_C, 
                            telemetry.volts_cV/100, telemetry.volts_cV%100, telemetry.amps_A, 
                            (float)telemetry.errors*100.0f/telemetry.reads);
        }
        Serial2.println(radio.isLinkUp());
    }
}
