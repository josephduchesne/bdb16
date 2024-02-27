/*
  Arduino LSM6DS3 - Simple Accelerometer

  This example reads the acceleration values from the LSM6DS3
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Uno WiFi Rev 2 or Arduino Nano 33 IoT

  created 10 Jul 2019
  by Riccardo Rizzo

  This example code is in the public domain.
*/
#include <Wire.h>
#include <IMUReader.h>
#include <etl/delegate.h>

IMUReader imu_reader;

void setup() {
  Serial2.begin(115200);
  delay(500);

  if (!imu_reader.start(IMUDelegate::create<IMUReader, &IMUReader::printData>(imu_reader))) {
    Serial2.println("Failed to initialize IMU!");

    while (1);
  }

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {

  if (imu_reader.read_loop()) {
    // Serial2.printf("int: %d\n", imu_reader.int_triggers);
    //imu_reader.printData(imu_reader.data);
    // IMU.readAcceleration(x, y, z);

    // Serial2.print(x);
    // Serial2.print('\t');
    // Serial2.print(y);
    // Serial2.print('\t');
    // Serial2.println(z);

    // analogWrite(LED_BUILTIN, (uint16_t)(z*255.0));
  }
}