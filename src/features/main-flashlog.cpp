#include <Arduino.h>

#include <FlashLog.h>

void setup() {
    Serial2.setFIFOSize(256);
    Serial2.begin(2000000);
    pinMode(LED_BUILTIN, OUTPUT);
    delay(500);

    FlashLog::Setup(true);

    long begin = micros();
    for(int i=0; i<14; i++) {
        FlashLog::WriteBasic(16200-i*100, i>7, {(uint16_t)1100+i, 1200, 1300, 500, 1000, 1500}, {200, 300, 400, 500});
    }
    for(int i=0; i<7; i++) {
        FlashLog::WriteESC(12340+i, i, i*100, 25+i, 150+i, 2+i);
    }
    for(int i=0; i<7; i++) {
        FlashLog::WriteIMU({1,2,i}, {-1, -2, -i});
    }
    auto duration_us = micros()-begin;

    printf("Wrote 28 flashlog entries in %luus, %dkB/s\n", duration_us, FLASH_PAGE_SIZE*2*1000/duration_us);

    //FlashLog::ReadAll();

}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(900);
}
