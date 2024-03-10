#include <Arduino.h>
#include <bdb16.h>

#include <Deneyap_6EksenAtaletselOlcumBirimi.h>       // Deneyap_IvmeOlcerVeDonuOlcer.h kütüphanesi eklendi

LSM6DSM AccGyro;                                      // AccGyro icin Class tanimlamasi

void setup() {
    // Serial2.begin(115200);                             // Seri haberleşme başlatıldı
    BDB16::init();
    if(AccGyro.begin(0x6A)==IMU_SUCCESS) Serial2.println("IMU Init Success");                              // begin(slaveAdress) fonksiyonu ile cihazların haberleşmesi başlatılması
    else Serial2.println("IMU Init Error");
}

void loop() {
    Serial2.printf("Acc: %.2f, %.2f, %.2f\t", AccGyro.readFloatAccelX(), AccGyro.readFloatAccelY(), AccGyro.readFloatAccelZ());
    Serial2.printf("Acc: %.2f, %.2f, %.2f\t", AccGyro.readFloatGyroX(), AccGyro.readFloatGyroY(), AccGyro.readFloatGyroZ());
    Serial2.printf("Temp: %.2fC %d %d\n", AccGyro.readTempC(), AccGyro.allOnesCounter, AccGyro.nonSuccessCounter);
    delay(100);
}