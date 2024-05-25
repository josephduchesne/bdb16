#pragma once

#include <cstdint>

class Encoder {
public:
    Encoder(uint8_t cs_pin, uint32_t min_val, uint32_t max_val, bool absolute=false);

    void init();

    float percent(int32_t val) const;
    float percent() const;
    int32_t get() const;

    int32_t update();

    // config
    int32_t min_val_;
    int32_t max_val_;
    const uint8_t cs_pin_;
    bool first_read = true;
    bool absolute_;

    // state
    int32_t zero_crossings_ = 0;
    int32_t last_value_ = -1;

};