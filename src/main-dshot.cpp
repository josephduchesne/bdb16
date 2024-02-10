#include <Arduino.h>
#include <dshot/esc.h>
#include <functional>   // std::reference_wrapper


// todo: move pole pair to 2nd argument
DShot::ESC dshotL(DS1, pio0, DShot::Type::Bidir, DShot::Speed::DS600, 14);
DShot::ESC dshotR(DS3, pio0, DShot::Type::Bidir, DShot::Speed::DS600, 14);
DShot::ESC dshotW(DS2, pio0, DShot::Type::Bidir, DShot::Speed::DS600, 14);
DShot::ESC& dshot = dshotR;
std::reference_wrapper<DShot::ESC> escs[] = {dshotL, dshotR};

int isr_count0 = 0;
int isr_count1 = 0;
int isr_count2 = 0;


static volatile bool alarm_fired;
bool repeating_timer_callback(struct repeating_timer *t) {
    Serial2.printf("Repeat at %lld\n", time_us_64());
    return true;
}

struct repeating_timer timer;

void setup() {
    Serial2.begin(115200);

    for(auto& esc : escs) {
        if (!esc.get().init()) {
            Serial2.println("ESC init error :/");
        } else {
            Serial2.printf("ESC init success on SM %d :)\n", esc.get().pio_sm);
        }
    }



    // Negative delay so means we will call repeating_timer_callback, and call it again
    // 500ms later regardless of how long the callback took to execute
    
    add_repeating_timer_ms(-500, repeating_timer_callback, NULL, &timer);
}


// the loop routine runs over and over again forever:
void loop() {
    static uint64_t last_print = 0;
    
    //Serial2.printf("PC0: %d, PC1: %d, isr cnt %d %d\n", pio_sm_get_pc(pio0, 0), pio_sm_get_pc(pio0, 1), isr_count0, isr_count1);

    delay(1);
    
    for(auto& esc : escs) {
        if (millis()< 6000) esc.get().setCommand(0);  // 1046 is the example command
        else if (millis() < 7000) esc.get().setCommand(13);  // extended telemetry enable
        else esc.get().setThrottle(0.10); // https://github.com/betaflight/betaflight/issues/2879
    }
    
    delay(2);

    // decode any incoming telemetry
    DShot::ESC::processTelemetryQueue();

    // todo: capture last telemetry time
    if (millis()-last_print >= 250) {
        last_print = millis();
        for(auto& esc : escs) {
            DShot::Telemetry& telemetry = esc.get().telemetry;
            Serial2.printf("%d: %drpm, %dC, %02d.%02dV, %dA %0.3f\t", esc.get().pio_sm, telemetry.rpm, telemetry.temperature_C, 
                            telemetry.volts_cV/100, telemetry.volts_cV%100, telemetry.amps_A, 
                            (float)telemetry.errors*100.0f/telemetry.reads);
        }
        Serial2.println();
    }
}
