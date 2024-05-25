#include <encoder.h>

#include <Arduino.h>
#include <SPI.h>

Encoder::Encoder(uint8_t cs_pin, uint32_t min_val, uint32_t max_val, bool absolute) 
: 
    cs_pin_(cs_pin),
    min_val_(min_val),
    max_val_(max_val),
    absolute_(absolute)
{

}

void Encoder::init() {
    SPI1.begin();
    pinMode(cs_pin_, OUTPUT);
    digitalWrite(cs_pin_, HIGH);
}

float Encoder::percent(int32_t val) const {
    return (float)(val-min_val_)/(float)(max_val_-min_val_)*100.0f;
}

float Encoder::percent() const {
    return percent(get());
}

int32_t Encoder::get() const {
    return last_value_ + 16384 * zero_crossings_;
}

int32_t Encoder::update() {
    digitalWrite(cs_pin_, LOW);
    SPI1.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));
    uint16_t result = SPI1.transfer16(0);
    //result = (( result ) & (0x3FFF));  // ignore MSB and MSB-1 (parity and 0)
    // todo: Check parity bits?
    SPI1.endTransaction();
    digitalWrite(cs_pin_, HIGH);
    //Serial2.printf("Encoder Value 0x%04x\n", result & 0x3FFF);
    result = result & 0x3FFF; //result;

    // if absolute, just return the raw result
    if (absolute_) {
        return result;
    }

    // account for zero crossings
    
    int32_t delta;
    if (first_read) {  // account for initialization
        delta = -(int32_t)result; 
        // if (delta<-8192) zero_crossings_--;
        first_read = false;
    } else {
        delta = last_value_-(int32_t)result;
        // ~180 degrees in one delta, indicates zero crossing
        if (delta>8191) zero_crossings_++;
        if (delta<-8191) zero_crossings_--;
    }

    last_value_ = (int32_t) result;
    return last_value_ + 16384 * zero_crossings_;
}
