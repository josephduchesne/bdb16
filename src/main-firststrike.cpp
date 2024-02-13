#include <Arduino.h>
#include <dshot/esc.h>
#include <functional>   // std::reference_wrapper
#include <CrsfSerial.h>

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
volatile float left, right;
bool repeating_timer_callback(struct repeating_timer *t) {
    // Serial2.printf("Repeat at %lld\n", time_us_64());
    if (millis()>=7000) {
        dshotL.setThrottle3D(left);
        dshotR.setThrottle3D(right);
    }
    return true;
}

struct repeating_timer timer;

CrsfSerial crsf(Serial1, CRSF_BAUDRATE);

unsigned long packets = 0;
void packetChannels() {packets++; }

void setup() {
    Serial2.begin(115200);

    for(auto& esc : escs) {
        if (!esc.get().init()) {
            Serial2.println("ESC init error :/");
        } else {
            Serial2.printf("ESC init success on SM %d :)\n", esc.get().pio_sm);
        }
    }


    crsf.onPacketChannels = &packetChannels;
    Serial1.setFIFOSize(64);
    Serial1.begin(CRSF_BAUDRATE, SERIAL_8N1);
    // Negative delay so means we will call repeating_timer_callback, and call it again
    // 500ms later regardless of how long the callback took to execute
    
    add_repeating_timer_ms(-2, repeating_timer_callback, NULL, &timer);
}

#define OUTPUT_MAX_RANGE 1.0f
#define INPUT_MAX_RANGE 500.0f
void ArcadeToDifferential(int drive, int steer, float &left, float &right) {
    if(drive>0) drive -= 1500;
    if(steer>0) steer -= 1500;

    // Channel mixing math from http://home.kendra.com/mauser/joystick.html
    // Get X and Y from the Joystick, do whatever scaling and calibrating you need to do based on your hardware.
    float x = (float)steer;
    float y = (float)drive;

    float v = (INPUT_MAX_RANGE - abs(x)) * (y / INPUT_MAX_RANGE) + y;
    float w = (INPUT_MAX_RANGE - abs(y)) * (x / INPUT_MAX_RANGE) + x;

    right = constrain((float)(((v + w) / 2.0)*OUTPUT_MAX_RANGE/INPUT_MAX_RANGE), -OUTPUT_MAX_RANGE, OUTPUT_MAX_RANGE);
    left = constrain((float)(((v - w) / 2.0)*OUTPUT_MAX_RANGE/INPUT_MAX_RANGE), -OUTPUT_MAX_RANGE, OUTPUT_MAX_RANGE);;
}

// the loop routine runs over and over again forever:
void loop() {
    static uint64_t last_print = 0;
    
    //Serial2.printf("PC0: %d, PC1: %d, isr cnt %d %d\n", pio_sm_get_pc(pio0, 0), pio_sm_get_pc(pio0, 1), isr_count0, isr_count1);

    crsf.loop();

    delay(1);
    
    // TODO: merge in QueenBee repo's DifferentialRobot/Subclass system
    // Arm ESCs (todo: disable music to speed up arming time)
    if (millis() < 7000) {
        for(auto& esc : escs) {
            if (millis()< 6000) esc.get().setCommand(0);  // 1046 is the example command
            else if (millis() < 7000) esc.get().setCommand(13);  // extended telemetry enable
        }
    } else {
        // do drive stuff
        if(crsf.isLinkUp()) {
            // handle the arcade steering on channels 1 and 2
            
            int ch1_in, ch2_in;
            ch1_in = crsf.getChannel(1);
            ch2_in = crsf.getChannel(2);
            if (ch1_in > 1485 && ch1_in < 1515) ch1_in = 1500;  // Add a small deadband
            if (ch2_in > 1485 && ch2_in < 1515) ch2_in = 1500;  // Add a small deadband
            float local_l, local_r;
            ArcadeToDifferential(ch1_in, ch2_in, local_l, local_r);
            if (millis()-last_print >= 250) {
                Serial2.printf("%d %d -> L: %f R: %f\n", ch1_in, ch2_in, local_l, local_r);
            }
            left = local_l;
            right = local_r;
        } else {
            left = 0;
            right = 0;
        }
    }
    
    delay(2);

    // decode any incoming telemetry
    DShot::ESC::processTelemetryQueue();

    // Todo: Set ESC on timer
    // Todo: Timeout ESC if not set for too long
    // Todo: Capture last telemetry time / time out telemetry validity

    if (millis()-last_print >= 250) {
        last_print = millis();
        for(auto& esc : escs) {
            DShot::Telemetry& telemetry = esc.get().telemetry;
            // Serial2.printf("%d: %drpm, %dC, %02d.%02dV, %dA %0.3f\t", esc.get().pio_sm, telemetry.rpm, telemetry.temperature_C, 
            //                 telemetry.volts_cV/100, telemetry.volts_cV%100, telemetry.amps_A, 
            //                 (float)telemetry.errors*100.0f/telemetry.reads);
        }
        //Serial2.println(crsf.isLinkUp());
    }
}
