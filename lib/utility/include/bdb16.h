#pragma once

#include <Arduino.h>
#include <array>
#include <Robot.h>

namespace BDB16 {

    // Reboots if no radio link or no ESC telemetry has been seen for this long
    inline constexpr uint32_t WATCHDOG_TIMEOUT_MS = 3000;
    // How often the watchdog condition is checked
    inline constexpr uint32_t WATCHDOG_CHECK_INTERVAL_MS = 100;

    inline Robot* watchdog_robot_ = nullptr;
    inline struct repeating_timer watchdog_timer_;
    inline uint32_t watchdog_last_radio_ms_ = 0;
    inline uint32_t watchdog_last_esc_ms_ = 0;
    inline std::array<uint16_t, OUTPUT_CHANNELS> watchdog_esc_reads_ = {0};

    inline bool __isr watchdog_callback(struct repeating_timer *t) {
        uint32_t now = millis();

        if (watchdog_robot_->radio_.isLinkUp()) {
            watchdog_last_radio_ms_ = now;
        }

        // telemetry.reads only advances when an ESC returns feedback
        bool esc_activity = false;
        for (size_t i = 0; i < watchdog_robot_->escs_.size(); i++) {
            uint16_t reads = watchdog_robot_->escs_[i].telemetry.reads;
            if (reads != watchdog_esc_reads_[i]) esc_activity = true;
            watchdog_esc_reads_[i] = reads;
        }
        if (esc_activity) watchdog_last_esc_ms_ = now;

        if (now - watchdog_last_radio_ms_ > WATCHDOG_TIMEOUT_MS ||
            now - watchdog_last_esc_ms_ > WATCHDOG_TIMEOUT_MS) {
            rp2040.reboot();  // soft reset via the hardware watchdog
        }

        return true;
    }

    inline void watchdog_init(Robot& robot) {
        watchdog_robot_ = &robot;
        watchdog_last_radio_ms_ = millis();
        watchdog_last_esc_ms_ = millis();
        watchdog_esc_reads_.fill(0);
        add_repeating_timer_ms(-(int32_t)WATCHDOG_CHECK_INTERVAL_MS, watchdog_callback, NULL, &watchdog_timer_);
    }

    void init() {
        Serial2.begin(2000000);
        delay(500); // slight pause to ensure Serial2 is connected PC side
        Serial2.printf("\u001b[33mBDB16\u001b[0m Online!\n");

        analogReadResolution(12);
        pinMode(PIN_VSENSE, INPUT);
    }

    void init(Robot& robot) {
        Serial2.begin(2000000);
        delay(500); // slight pause to ensure Serial2 is connected PC side
        Serial2.printf("\u001b[33m%s\u001b[0m Online!\n", robot.name());

        analogReadResolution(12);
        pinMode(PIN_VSENSE, INPUT);

        watchdog_init(robot);
    }

    uint16_t read_voltage_mV() {
        static uint64_t era_mV = 0;

        // will be a 12 bit number (0-4095) where max input is 3.3V pr 3300mV
        uint64_t value = (uint64_t)analogRead(PIN_VSENSE) * 110u * 3300u / 10u / 4096u; 

        if (era_mV == 0) {
            era_mV = value;
        } else {
            era_mV = (era_mV * 90 + value * 10) / 100;
        }
        return (uint16_t) era_mV;
        
    }
}
