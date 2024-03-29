#pragma once

#include <Deneyap_6EksenAtaletselOlcumBirimi.h>
#include <etl/delegate.h>


typedef struct __attribute__((__packed__)) 
{
    uint64_t timestamp_us;
    int16_t gyro[3];     // 6 bytes
    int16_t accel[3];    // 6 bytes
} IMUData;

typedef etl::delegate<void(IMUData&)> IMUDelegate;

class IMUReader {
public:
    IMUReader();

    bool start(IMUDelegate _data_callback = IMUDelegate());
    static void imu_isr();
    uint8_t read_loop();
    void printData(IMUData& data);

    volatile uint32_t int_triggers = 0;
private:
    volatile uint32_t imu_fifo_ready = 0;
    IMUDelegate data_callback;

public:
    IMUData data; // todo: Probably needs a mutex on data access?

    volatile uint32_t fifo_triggers;
    volatile uint32_t not_empty_count;
    static IMUReader* instance_;

};
