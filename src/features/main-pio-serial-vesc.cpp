#include <Arduino.h>
#include <bdb16.h>

#include <SoftwareSerial.h>
#include <VescUart.h>

SoftwareSerial vesc_serial(/* rx */ 0, /* tx */ 1);
VescUart vesc(10 /*ms timeout*/);


void setup() {
  BDB16::init();

  pinMode(LED_BUILTIN, OUTPUT);

  vesc_serial.begin(115200);

  vesc.setSerialPort(&vesc_serial);
  // vesc.setDebugPort(&Serial2);

  Serial2.printf("eRPM V A pidPos rxd millis\n");
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);

  if (millis() < 1000) {
    vesc.setCurrent(40.0);
  } else {
    vesc.setCurrent(0);
  }

  auto start = micros();
  bool success = vesc.getVescValues();
  auto end = micros()-start;

  if ( success ) {
    Serial2.printf("%.2f %.2f %.2f %.2f %lu %lu\n", vesc.data.rpm, vesc.data.inpVoltage, vesc.data.avgMotorCurrent, vesc.data.pidPos, end, millis());
  }
  else
  {
    Serial2.printf("Failed to get data! %lu\n", end);
  }

  delay(1);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1);

  if (millis() >2000) {
    Serial2.println("done");
    while(1){}
  }
}
