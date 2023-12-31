#pragma once

// Pin definitions taken from:
//    https://datasheets.raspberrypi.org/pico/pico-datasheet.pdf


// LEDs
#define PIN_LED        (27u)

// IO pins
static const uint8_t IO0 = (0u);
static const uint8_t IO1 = (1u);
static const uint8_t IO2 = (2u);
static const uint8_t IO3 = (3u);
static const uint8_t IO4 = (4u);
static const uint8_t IO5 = (5u);

// DShot pins
static const uint8_t DS0 = (6u);
static const uint8_t DS1 = (7u);
static const uint8_t DS2 = (8u);
static const uint8_t DS3 = (9u);

// Board function specific pins
#define PIN_RGB (19u)
#define IMU_INT1 (23u)
#define PIN_SPI0_CS0 (21u)
#define PIN_SPI0_CS1 (20u)
#define PIN_SPI0_CS2 (18u)

// Serial
#define PIN_SERIAL1_TX (16u)
#define PIN_SERIAL1_RX (17u)

#define PIN_SERIAL2_TX (4u)
#define PIN_SERIAL2_RX (5u)

// SPI
#define PIN_SPI0_MISO  (12u)
#define PIN_SPI0_MOSI  (11u)
#define PIN_SPI0_SCK   (10u)
#define PIN_SPI0_SS    PIN_SPI0_CS0

// #define PIN_SPI1_MISO  (12u)
// #define PIN_SPI1_MOSI  (15u)
// #define PIN_SPI1_SCK   (14u)
// #define PIN_SPI1_SS    (13u)

// Wire
#define PIN_WIRE0_SDA  (24u)
#define PIN_WIRE0_SCL  (25u)

#define PIN_WIRE1_SDA  (24u)
#define PIN_WIRE1_SCL  (25u)

#define SERIAL_HOWMANY (2u)
#define SPI_HOWMANY    (1u)
#define WIRE_HOWMANY   (1u)

#include "common.h"
