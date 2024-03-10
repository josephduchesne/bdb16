#pragma once

#include <Arduino.h>
#include <array>

extern "C" {
#include <hardware/sync.h>
#include <hardware/flash.h>
};

extern uint8_t _FS_start;
extern uint8_t _FS_end;


#define FLASH_LOG_START ((uint32_t)&_FS_start) 
#define FLASH_LOG_END ((uint32_t)&_FS_end) 
#define FLASH_LOG_SIZE (FLASH_LOG_END - FLASH_LOG_START)

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

typedef struct __attribute__((__packed__)) IMULog3 {
    int16_t gyro[3];     // 6 bytes
    int16_t accel[3];    // 6 bytes
} IMULog3;

// Page is 256 bytes, so we should store in a way that fits N logs within 252 bytes (4 for packet ID)
typedef struct __attribute__((__packed__)) LogData {
    uint8_t packetType;
    uint32_t timestamp;    // 1/256ms resolution, 2^20 = 1048576ms >= 17min log time before rollover
    union {
        BasicLog1 basic_log_1;
        ESCLog2 esc_log_2;
        IMULog3 imu_log_3;
    };
} LogEntry;

// Each log payload should be N bytes log (allowing us to fit (256-4)/(N) = PPB logs per block)
constexpr size_t packets_per_block = (FLASH_PAGE_SIZE-4)/(sizeof(LogEntry));

typedef struct LogPage {
  unsigned int packet;
  LogEntry data[packets_per_block];
} LogPage;


void FindFirstFreeSector();

void EraseEverything();

LogEntry* GetNextEntry();
void WritePacketIfFull();

void WriteBasic(uint16_t battery_mV, bool radio_connected, const std::array<uint16_t, 6>& rc, const std::array<uint16_t, 4> out);

void WriteESC(uint64_t timestamp_us, uint8_t esc, 
                      uint32_t rpm, uint8_t temperature_C, 
                      uint16_t volts_cV, uint8_t amps_A);

void WriteIMU(const std::array<int16_t, 3>& gyro, const std::array<int16_t, 3>& accel);

void PrintDecimal8Bit(uint32_t val);
void PrintDecimal4Bit(uint16_t val);

void PrintPacket(uint32_t page, uint8_t packet, LogEntry& entry);

void ReadAll();

void Setup(bool skip_prompt=false);

}  // end namespace FlashLog