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
#include <Arduino_LSM6DS3.h>

//LSM6DS3Class IMU_LSM6DS3(Wire, LSM6DS3_ADDRESS);

void setup() {
  Serial1.begin(115200);

  if (!IMU.begin()) {
    Serial1.println("Failed to initialize IMU!");

    while (1);
  }

  pinMode(LED_BUILTIN, OUTPUT);
  analogWriteResolution(8);

  Serial1.print("Accelerometer sample rate = ");
  Serial1.print(IMU.accelerationSampleRate());
  Serial1.println(" Hz");
  Serial1.println();
  Serial1.println("Acceleration in g's");
  Serial1.println("X\tY\tZ");
}

void loop() {
  float x, y, z;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    Serial1.print(x);
    Serial1.print('\t');
    Serial1.print(y);
    Serial1.print('\t');
    Serial1.println(z);

    analogWrite(LED_BUILTIN, (uint16_t)(z*255.0));
  }
}