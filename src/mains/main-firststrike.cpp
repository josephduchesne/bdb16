#include <Arduino.h>
#include <robots/DifferentialRobot.h>
#include <dshot/esc.h>
#include <functional>   // std::reference_wrapper
#include <CrsfSerial.h>

// todo: move pole pair to 2nd argument

static volatile bool alarm_fired;
volatile float left, right;

struct repeating_timer timer;

CrsfSerial radio(Serial1, CRSF_BAUDRATE);

DifferentialRobot robot( radio, {
            DShot::ESC(DS1, pio0, DShot::Type::Bidir, DShot::Speed::DS600, 14), // left drive
            DShot::ESC(DS3, pio0, DShot::Type::Bidir, DShot::Speed::DS600, 14), // right drive
            DShot::ESC(DS2, pio0, DShot::Type::Bidir, DShot::Speed::DS600, 14)  // weapon
        }
    );

unsigned long packets = 0;
void packetChannels() {packets++; }

bool repeating_timer_callback(struct repeating_timer *t) {
    // Serial2.printf("Repeat at %lld\n", time_us_64());
    robot.update();
    // todo: swap for output?
    
    return true;
}

void setup() {
    Serial2.begin(115200);

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
    
    //Serial2.printf("PC0: %d, PC1: %d, isr cnt %d %d\n", pio_sm_get_pc(pio0, 0), pio_sm_get_pc(pio0, 1), isr_count0, isr_count1);

    delay(1);
    // todo: Put update()

    // Todo: Set ESC on timer
    // Todo: Timeout ESC if not set for too long
    // Todo: Capture last telemetry time / time out telemetry validity

    if (millis()-last_print >= 250) {
        last_print = millis();
        for(auto& esc : robot.escs_) {
            DShot::Telemetry& telemetry = esc.telemetry;
            // Serial2.printf("%d: %drpm, %dC, %02d.%02dV, %dA %0.3f\t", esc.get().pio_sm, telemetry.rpm, telemetry.temperature_C, 
            //                 telemetry.volts_cV/100, telemetry.volts_cV%100, telemetry.amps_A, 
            //                 (float)telemetry.errors*100.0f/telemetry.reads);
        }
        //Serial2.println(crsf.isLinkUp());
    }
}
