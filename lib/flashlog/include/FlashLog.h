#pragma once

#include <Arduino.h>
#include <array>

extern "C" {
#include <hardware/sync.h>
#include <hardware/flash.h>
};

#define FLASH_PROGRAM_SPACE 524288
#define FLASH_TARGET_OFFSET FLASH_PROGRAM_SPACE
#define FLASH_TOTAL_STORAGE (PICO_FLASH_SIZE_BYTES - FLASH_PROGRAM_SPACE)

namespace FlashLog {

constexpr uint64_t BITMASK_4 = ((1<<4) - 1);
constexpr uint64_t BITMASK_10 = ((1<<10) - 1);
constexpr uint64_t BITMASK_11 = ((1<<11) - 1);
constexpr uint64_t BITMASK_20 = ((1<<20) - 1);

typedef struct __attribute__((__packed__)) BasicLog1 {
    uint16_t battery : 10;  // 10 bit resolution. range: 0-64V, increment 0.0625V
    uint8_t radio_connected : 1;  // single bit flag
    uint16_t rc_ch1 : 11;
    uint16_t rc_ch2 : 11;
    uint16_t rc_ch3 : 11;
    uint8_t rc_ch4 : 4;
    uint8_t rc_ch5 : 4;
    uint8_t rc_ch6 : 4;
    uint16_t out0 : 11;
    uint16_t out1 : 11;
    uint16_t out2 : 11;
    uint16_t out3 : 11;
} BasicLog1;

typedef struct __attribute__((__packed__)) ESCLog2 {
    uint8_t esc;            // ESC offset id (0-3 for PIO dshot)
    uint32_t rpm;               // rotations per minute, assuming pole count is correct
    uint8_t temperature_C;      // degrees Celcius
    uint16_t volts_cV;          // centi-volts, or V/100
    uint8_t amps_A;             // Amps
} ESCLog2;

// Page is 256 bytes, so we should store in a way that fits N logs within 252 bytes (4 for packet ID)
typedef struct __attribute__((__packed__)) LogData {
    uint8_t packetType;
    uint32_t timestamp;    // 1/256ms resolution, 2^20 = 1048576ms >= 17min log time before rollover
    union {
        BasicLog1 basic_log_1;
        ESCLog2 esc_log_2;
        // TODO: imu_log_3
    };
} LogEntry;

// Each log payload should be N bytes log (allowing us to fit (256-4)/(N) = PPB logs per block)
constexpr size_t packets_per_block = (FLASH_PAGE_SIZE-4)/(sizeof(LogEntry));

typedef struct LogPage {
  unsigned int packet;
  LogEntry data[packets_per_block];
} LogPage;


LogPage buf;  // One page buffer of bytes
unsigned int current_flash_offset = FLASH_PROGRAM_SPACE;
unsigned int current_packet = 1;
unsigned int current_sector_packet = 0;


void FlashLogFindFirstFreeSector() {
  int blocks_checked = 0;
  for (unsigned int i = FLASH_TARGET_OFFSET; i < PICO_FLASH_SIZE_BYTES; i += FLASH_PAGE_SIZE) {
    blocks_checked++;
    if ( -1 == *((int *)(XIP_BASE + i)) ) {
      Serial2.printf("Free block found at %d (%d blocks searched)\n", i, blocks_checked);
      current_flash_offset = i;
      return;
    }
  }
  Serial2.printf("Flash is full\n");
  current_flash_offset = PICO_FLASH_SIZE_BYTES;  // we're full
}


void FlashLogEraseEverything() {
  int sector = 0;
  for (unsigned int i = FLASH_TARGET_OFFSET; i < PICO_FLASH_SIZE_BYTES; i += FLASH_SECTOR_SIZE) {
    long start = micros();
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(i, FLASH_SECTOR_SIZE);
    restore_interrupts (ints);
    Serial2.printf("Erased sector %d in %d us\n", sector, micros() - start);
    sector++;
  }
  current_flash_offset = FLASH_PROGRAM_SPACE;
}

void FlashLogWritePacket(LogEntry &packet) {

  if (current_flash_offset >= PICO_FLASH_SIZE_BYTES) {
    Serial2.println("Flash full");
    return;
  }
  
  if (current_sector_packet == 0) {  // first packet
    buf.packet = current_packet;
    //Serial2.printf("Starting fresh FlashLog page %d\n", buf.packet);
  }

  //Serial2.printf("Copying packet in to %d\n", current_sector_packet);
  memcpy(&buf.data[current_sector_packet], &packet, sizeof(LogEntry));

  if (current_sector_packet < packets_per_block) {
    current_sector_packet++;
  } else { // full, write out now!
    Serial2.printf("Filled FlashLog page %d, writing to %d\n", buf.packet, current_flash_offset);
    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(current_flash_offset, (uint8_t *)(&buf), FLASH_PAGE_SIZE);
    restore_interrupts (ints);
    //Serial2.println("Done writing!");
    current_flash_offset += FLASH_PAGE_SIZE;
    current_sector_packet = 0;
    current_packet++;
  }
}

// FlashLogWriteBasic
void FlashLogWriteBasic(uint16_t battery_mV, const std::array<uint16_t, 6>& rc, const std::array<uint16_t, 4> out) {
    static LogEntry entry;
    
    entry.packetType = 1;  // todo: Enum!
    entry.timestamp = (uint32_t)(micros() * 256/1000);  // convert from us to 1/256ms

    BasicLog1& bl1 = entry.basic_log_1;
    bl1.battery = (((uint64_t)battery_mV) * 16 / 1000) & BITMASK_10; // 10 bit range
    bl1.rc_ch1 = rc[0] & BITMASK_11; // 11 bit in/out
    bl1.rc_ch2 = rc[1] & BITMASK_11; // 11 bit in/out
    bl1.rc_ch3 = rc[2] & BITMASK_11; // 11 bit in/out
    bl1.rc_ch4 = (rc[3] >> 7) & BITMASK_4; // 11 bit in/ 4 bit out
    bl1.rc_ch5 = (rc[4] >> 7) & BITMASK_4; // 11 bit in/ 4 bit out
    bl1.rc_ch6 = (rc[5] >> 7) & BITMASK_4; // 11 bit in/ 4 bit out
    bl1.out0 = out[0] & BITMASK_11; // 11 bit in/out
    bl1.out1 = out[1] & BITMASK_11; // 11 bit in/out
    bl1.out2 = out[2] & BITMASK_11; // 11 bit in/out
    bl1.out3 = out[3] & BITMASK_11; // 11 bit in/out
    FlashLogWritePacket(entry);
}

// FlashLogWriteESC
void FlashLogWriteESC(uint64_t timestamp_us, uint8_t esc, 
                      uint32_t rpm, uint8_t temperature_C, 
                      uint16_t volts_cV, uint8_t amps_A) {
    static LogEntry entry;
    entry.packetType = 2; // todo: Enum!
    entry.timestamp = (uint32_t)(timestamp_us * 256/1000);  // convert from us to ms/256

    ESCLog2& el2 = entry.esc_log_2;

    el2.esc = esc & BITMASK_4;  // output restricted to 4 bits
    el2.rpm = rpm;
    el2.temperature_C = temperature_C;
    el2.volts_cV = volts_cV;
    el2.amps_A = amps_A;

    FlashLogWritePacket(entry);
}

void FlashLogPrintDecimal8Bit(uint32_t val) {
    Serial2.printf("%d.%4d ", val>>8, (uint16_t)((uint64_t)(val & 0b11111111)*1000/256));
}
void FlashLogPrintDecimal4Bit(uint16_t val) {
    Serial2.printf("%d.%4d ", val>>4, 625*(uint16_t)(val & 0b1111));
}

void FlashLogPrintPacket(uint32_t page, uint8_t packet, LogEntry& entry) {
    Serial2.printf("%d %d %d ", page, packet, entry.packetType);
    switch (entry.packetType) {
        case 1:  // BasicLog1
            BasicLog1& bl1 = entry.basic_log_1;
            Serial2.printf("%u ", bl1.timestamp);
            FlashLogPrintDecimal4Bit(bl1.battery);
            Serial2.printf("%u ", bl1.radio_connected);
            Serial2.printf("%u %u %u", bl1.rc_ch1, bl1.rc_ch2, bl1.rc_ch3);
            // convert 4 bit back to 11 bit
            Serial2.printf("%u %u %u", bl1.rc_ch3 << 7, bl1.rc_ch4 << 7, bl1.rc_ch5 << 7);
            Serial2.printf("%u %u %u %u\n", bl1.out0, bl1.out1, bl1.out2, bl1.out3);
            break;
        case 2: // ESCLog2
            ESCLog2& el2 = entry.esc_log_2;
            Serial2.printf("%u %u ", el2.esc, el2.timestamp);
            Serial2.printf("%u %u %u.%2u %d\n", el2.rpm, el2.temperature_C, 
                            el2.volts_cV/100, el2.volts_cV%100, el2.amps_A);
            break;
        default:
            Serial2.println("Unknown packet type");
    }
}

void FlashLogReadAll() {
  LogPage *p;
  Serial2.println("> Read start");
  for (unsigned int i = FLASH_TARGET_OFFSET; i < PICO_FLASH_SIZE_BYTES; i += FLASH_PAGE_SIZE) {
    if ( -1 == *((int *)(XIP_BASE + i)) ) {
      Serial2.printf("> Finished reading flash\n", i);
      current_flash_offset = i;
      return;
    } else { // print the block
      p = (LogPage *)(XIP_BASE + i);
      Serial2.printf("> Page %d\n", p->packet);
      for (unsigned int j=0; j<packets_per_block; j++) {
        FlashLogPrintPacket(p->data[j]);
      }
    }
  }
  Serial2.printf("> Flash was full!\n");
}


void FlashLog_Setup() {
    
    Serial2.print("Flash init, max storage "); Serial2.print(FLASH_TOTAL_STORAGE / 1024); Serial2.println("kB");
    Serial2.println("FLASH_PAGE_SIZE = " + String(FLASH_PAGE_SIZE, DEC));
    Serial2.println("FLASH_SECTOR_SIZE = " + String(FLASH_SECTOR_SIZE, DEC));
    Serial2.println("FLASH_BLOCK_SIZE = " + String(FLASH_BLOCK_SIZE, DEC));
    Serial2.println("PICO_FLASH_SIZE_BYTES = " + String(PICO_FLASH_SIZE_BYTES, DEC));
    Serial2.println("XIP_BASE = 0x" + String(XIP_BASE, HEX));
    Serial2.printf("Page buffer size: %d\n", sizeof(LogPage));
    FlashLogFindFirstFreeSector();
    Serial2.println("Available commands:\n`R` to read out via serial\n`E` to erase flash\n `G` to resume program flow.");

    // Only USBSerial knows if it's connected, so we'll need a different flag, loopback IO0->IO1 on the dongle?
    // Test: set IO0 as output, IO1 as input.
    pinMode(IO0, OUTPUT);
    pinMode(IO1, INPUT_PULLDOWN);
    int loopback_test_pass = 0;

    // Write IO0 low, wait 1ms, read IO1 as low
    digitalWrite(IO0, LOW);
    delay(1);
    if (!digitalRead(IO1)) loopback_test_pass++;

    // Write IO0 high, wait 1ms, read IO1 as high
    digitalWrite(IO0, HIGH);
    delay(1);
    if (digitalRead(IO1)) loopback_test_pass++;

    // if this works, IO0/1 are bridged and debug connection is plugged in
    if (Serial2 && loopback_test_pass == 2) {
        Serial2.println("Debug connection found. Please enter a command.");
        while (!Serial2.available()) {  // hang, waiting for a response
            delay(1);
        }
        char incomingByte = (char)Serial2.read();

        switch (incomingByte) {
            case 'R':
            case 'r':
                FlashLogReadAll();
                Serial2.println("> Read complete. Halting.");
                while (1);
                break;
            case 'E':
            case 'e':
                FlashLogEraseEverything();
                Serial2.println("Erase complete. Halting.");
                while (1);
                break;
            default:
                Serial2.println("Other character, resuming");
                break;
        }
    } else {
        Serial2.println("No debug connection found. How are you reading this?");
    }
}

}  // end namespace FlashLog