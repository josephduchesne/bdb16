#include "IMUReader.h"
#include <Arduino.h>

LSM6DSM imu;

IMUReader* IMUReader::instance_= nullptr;;

IMUReader::IMUReader() : 
    fifo_triggers(0),
    not_empty_count(0)
{
    IMUReader::instance_ = this; // singleton for ISR access :/
}

bool IMUReader::start(/*edl::delegate<void(IMUData& data)> _data_callback*/) {
    auto status = imu.begin(0x6A);

    //data_callback = _data_callback;

    if (status)  {
        Serial2.printf("> IMU Driver init failed: %d\n", status);
        return false;
    }

    imu.writeRegister(INT1_CTRL, INT1_FTH_ENABLED);


    pinMode(IMU_INT1, INPUT);
    attachInterrupt(digitalPinToInterrupt(IMU_INT1), &IMUReader::imu_isr, RISING);
    
    imu.fifoEnd();
    delay(5);
    // imu.fifoClear();

    imu.fifoBegin();

    Serial2.printf("> IMU Driver init success\n");

    return true;
}


void IMUReader::imu_isr() {
    if(IMUReader::instance_ != nullptr) {
        IMUReader::instance_->imu_fifo_ready = 1;
        IMUReader::instance_->int_triggers++;
    }
}

uint8_t IMUReader::read_loop() {
    if (!imu_fifo_ready) return 0;
    imu_fifo_ready = 0;  // possible race condition? Fifo should absorb it, but it's not ideal.

    uint8_t fifo_bytes;
    uint8_t blocks_read = 0;
    while(1) {
        imu.readRegister(FIFO_STATUS1, &fifo_bytes);
        if (fifo_bytes>=12) {
            // Serial2.printf("FIFO bytes: %u\n", fifo_bytes);
            fifo_triggers++;
            data.timestamp_us = micros();
            // probably should check the fifo=6 here?
            data.gyro[0] = imu.fifoRead();  // imu.calcGyro
            data.gyro[1] = imu.fifoRead();
            data.gyro[2] = imu.fifoRead();
            data.accel[0] = imu.fifoRead(); // imu.calcAccel
            data.accel[1] = imu.fifoRead();
            data.accel[2] = imu.fifoRead();
            if(imu.fifoGetStatus() & 0x1000) not_empty_count++;
            data_callback.call_if(data); // call data ready callback, if the callback is valid
            blocks_read++;
        } else {  // read all FIFO queue blocks
            break;
        }
    }
    return blocks_read;
}

void IMUReader::printData(IMUData& data) {
    Serial2.printf("gyro: <%1.3f. %1.3f, %1.3f> acc: <%1.3f, %1.3f, %1.3f> ts: %luus\n", 
        imu.calcGyro(data.gyro[0]), imu.calcGyro(data.gyro[1]), imu.calcGyro(data.gyro[2]),
        imu.calcAccel(data.accel[0]), imu.calcAccel(data.accel[1]), imu.calcAccel(data.accel[2]),
        data.timestamp_us
        );
}
