#include <bdb16.h>
#include <Encoder.h>

#define ENCODER_CS PIN_SPI0_CS0
Encoder encoder0(PIN_SPI0_CS0, 5879, 10759);  // right leg
Encoder encoder1(PIN_SPI0_CS1, 9298, 4353);  // left leg
Encoder encoder2(PIN_SPI0_CS2, 4375, 96650);  // hammer

void setup() {
    BDB16::init();
    encoder0.init();
    encoder1.init();
    encoder2.init();
}

void loop() {
    encoder0.update();
    encoder1.update();
    encoder2.update();
    Serial2.printf("%d %.2f %d %.2f %d %.2f\n", encoder0.get(), encoder0.percent(), encoder1.get(), encoder1.percent(), encoder2.get(), encoder2.percent());
    delay(2);
}