#include <Arduino.h>
#include <CrsfSerial.h>

CrsfSerial crsf(Serial1, CRSF_BAUDRATE);

unsigned long packets = 0;
void packetChannels() {packets++; }

void setup() {
  
  Serial2.begin(115200);

  crsf.onPacketChannels = &packetChannels;
  Serial1.setFIFOSize(64);
  Serial1.begin(CRSF_BAUDRATE, SERIAL_8N1);


}

long int last = 0;
void loop() {

    // if ((telemetry_counter_++)%telemetry_ratio_ == 0) {
    //     // battery telemetry
    //     crsf_sensor_battery_t crsfbatt = { 0 };
    //     crsfbatt.voltage = 321; // htobe16(batteryVoltage()/10);  // 3.5V
    //     crsfbatt.current = htobe16(123);
    //     radio_.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfbatt, sizeof(crsfbatt));
    // }
    crsf.loop();
    // Serial2.println(Serial1.available());
    // int bytes = 0;
    // while(Serial1.available()) {
    //     Serial1.read();
    //     bytes++;
    // }
    // Serial2.println(bytes);
    delay(1);

    if (millis()-last>100) {
        Serial2.print("Radio up: ");
        // Serial2.println("no");
        Serial2.print(crsf.isLinkUp());
        Serial2.print(" ch0: ");
        Serial2.print(crsf.getChannel(1));
        Serial2.print(" ");
        Serial2.print(crsf.getChannel(2));
        Serial2.print(" pkts: "); Serial2.println(packets);

        last = millis();
    }
}