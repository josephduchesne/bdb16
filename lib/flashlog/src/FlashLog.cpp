#include <FlashLog.h>

namespace FlashLog {

LogPage buf;  // One page buffer of bytes
unsigned int current_flash_offset = FLASH_LOG_START;
unsigned int current_packet = 1;
unsigned int current_sector_packet = 0;


void FindFirstFreeSector() {
  int blocks_checked = 0;
  for (unsigned int i = FLASH_LOG_START; i < FLASH_LOG_END; i += FLASH_PAGE_SIZE) {
    blocks_checked++;
    if ( -1 == *((int *)(i)) ) {
      Serial2.printf("Free block found at offset %d (%d blocks searched)\n", i-FLASH_LOG_START, blocks_checked);
      current_flash_offset = i;
      return;
    }
  }
  Serial2.printf("Flash is full\n");
  current_flash_offset = PICO_FLASH_SIZE_BYTES;  // we're full
}


void EraseEverything() {
  int sector = 0;
  for (unsigned int i = FLASH_LOG_START; i < FLASH_LOG_END; i += FLASH_SECTOR_SIZE) {
    long start = micros();
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(i-XIP_BASE, FLASH_SECTOR_SIZE);
    restore_interrupts (ints);
    Serial2.printf("Erased sector %d/%d in %d us\n", sector, FLASH_LOG_SIZE/FLASH_SECTOR_SIZE, micros() - start);
    sector++;
  }
  current_flash_offset = FLASH_LOG_START;
}

void WritePacket(LogEntry &packet) {

  if (current_flash_offset >= FLASH_LOG_END) {
    Serial2.println("Flash full");
    return;
  }
  
  if (current_sector_packet == 0) {  // first packet
    buf.packet = current_packet;
    // Serial2.printf("Starting fresh FlashLog page %d\n", buf.packet);
  }

  //Serial2.printf("Copying packet in to %d\n", current_sector_packet);
  memcpy(&buf.data[current_sector_packet], &packet, sizeof(LogEntry));

  if (current_sector_packet < packets_per_block-1) {
    current_sector_packet++;
  } else { // full, write out now!
    Serial2.printf("FlashLog wrote page %d\n", buf.packet);
    //Serial2.printf("Filled FlashLog page %d, writing to offset %d\n", buf.packet, current_flash_offset - XIP_BASE);
    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(current_flash_offset-XIP_BASE, (uint8_t *)(&buf), FLASH_PAGE_SIZE);
    restore_interrupts (ints);
    //Serial2.println("Done writing!");
    current_flash_offset += FLASH_PAGE_SIZE;
    current_sector_packet = 0;
    current_packet++;
  }
}

void WriteBasic(uint16_t battery_mV, bool radio_connected, const std::array<uint16_t, 6>& rc, const std::array<uint16_t, 4> out) {
    static LogEntry entry;
    
    entry.packetType = 1;  // todo: Enum!
    entry.timestamp = (uint32_t)(micros() * (uint64_t)256/(uint64_t)1000);  // convert from us to 1/256ms

    BasicLog1& bl1 = entry.basic_log_1;
    bl1.battery = (((uint64_t)battery_mV) * 16 / 1000) & BITMASK_10; // 10 bit range
    bl1.radio_connected = radio_connected & 0b1;
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
    WritePacket(entry);
}

void WriteESC(uint64_t timestamp_us, uint8_t esc, 
                      uint32_t rpm, uint8_t temperature_C, 
                      uint16_t volts_cV, uint8_t amps_A) {
    static LogEntry entry;
    entry.packetType = 2; // todo: Enum!
    entry.timestamp = (uint32_t)(timestamp_us * (uint64_t)256/(uint64_t)1000);  // convert from us to ms/256

    ESCLog2& el2 = entry.esc_log_2;

    el2.esc = esc & BITMASK_4;  // output restricted to 4 bits
    el2.rpm = rpm;
    el2.temperature_C = temperature_C;
    el2.volts_cV = volts_cV;
    el2.amps_A = amps_A;

    WritePacket(entry);
}

void WriteIMU(const std::array<int16_t, 3>& gyro, const std::array<int16_t, 3>& accel) {
    static LogEntry entry;
    
    entry.packetType = 3;  // todo: Enum!
    entry.timestamp = (uint32_t)(micros() * (uint64_t)256/(uint64_t)1000);  // convert from us to 1/256ms

    IMULog3& il3 = entry.imu_log_3;
    il3.gyro[0] = gyro[0];
    il3.gyro[1] = gyro[1];
    il3.gyro[2] = gyro[2];
    il3.accel[0] = accel[0];
    il3.accel[1] = accel[1];
    il3.accel[2] = accel[2];
    WritePacket(entry);
}

void PrintDecimal8Bit(uint32_t val) {
    Serial2.printf("%d.%04d ", val>>8, (uint16_t)((uint64_t)(val & 0b11111111)*1000/256));
}
void PrintDecimal4Bit(uint16_t val) {
    Serial2.printf("%d.%03d ", val>>4, 625*(uint16_t)(val & 0b1111));
}

void PrintPacket(uint32_t page, uint8_t packet, LogEntry& entry) {
    Serial2.printf("%d %d %d %u ", page, packet, entry.packetType, entry.timestamp);
    switch (entry.packetType) {
        case 1:  // BasicLog1
            {
              BasicLog1& bl1 = entry.basic_log_1;
              PrintDecimal4Bit(bl1.battery);
              Serial2.printf(" %u ", bl1.radio_connected);
              Serial2.printf("%u %u %u ", bl1.rc_ch1, bl1.rc_ch2, bl1.rc_ch3);
              // convert 4 bit back to 11 bit
              Serial2.printf("%u %u %u ", bl1.rc_ch4 << 7, bl1.rc_ch5 << 7, bl1.rc_ch6 << 7);
              Serial2.printf("%u %u %u %u\n", bl1.out0, bl1.out1, bl1.out2, bl1.out3);
            }
            break;
        case 2: // ESCLog2
          {
            ESCLog2& el2 = entry.esc_log_2;
            Serial2.printf("%u ", el2.esc);
            Serial2.printf("%u %u %u %d\n", el2.rpm, el2.temperature_C, 
                            el2.volts_cV, el2.amps_A);
          }
            break;
        case 3: // IMULog3
          {
            IMULog3& il3 = entry.imu_log_3;
            Serial2.printf("%d %d %d %d %d %d\n", il3.gyro[0], il3.gyro[1], il3.gyro[2], il3.accel[0], il3.accel[1], il3.accel[2]);
          }
            break;
        default:
            Serial2.println("Unknown packet type");
            break;
    }
}

void ReadAll() {
  LogPage *p;
  Serial2.println("> Read start");
  uint32_t page = 0;
  for (unsigned int i = FLASH_LOG_START; i < FLASH_LOG_END; i += FLASH_PAGE_SIZE) {
    if ( -1 == *((int *)i) ) {
      Serial2.printf("> Finished reading flash\n", i);
      current_flash_offset = i;
      return;
    } else { // print the block
      p = (LogPage *)i;
      Serial2.printf("> Page %d\n", p->packet);
      for (unsigned int j=0; j<packets_per_block; j++) {
        PrintPacket(page, j, p->data[j]);
      }
    }
    page++;
  }
  Serial2.printf("> Flash was full!\n");
}


void Setup() {
    
    Serial2.print("Flash init, max storage "); Serial2.print(FLASH_LOG_SIZE / 1024); Serial2.println("kB");
    Serial2.println("FLASH_PAGE_SIZE = " + String(FLASH_PAGE_SIZE, DEC));
    Serial2.println("FLASH_SECTOR_SIZE = " + String(FLASH_SECTOR_SIZE, DEC));
    Serial2.println("FLASH_BLOCK_SIZE = " + String(FLASH_BLOCK_SIZE, DEC));
    Serial2.println("PICO_FLASH_SIZE_BYTES = " + String(PICO_FLASH_SIZE_BYTES, DEC));
    Serial2.printf("FlashLog Max Entries = %d\n", FLASH_LOG_SIZE/FLASH_PAGE_SIZE*packets_per_block);
    Serial2.println("XIP_BASE = 0x" + String(XIP_BASE, HEX));
    Serial2.printf("Page buffer size: %d\n", sizeof(LogPage));
    FindFirstFreeSector();
    Serial2.println("Available commands:\n`R` to read out via serial\n`E` to erase flash\n `G` to resume program flow.");

    // Only USBSerial knows if it's connected, so we'll need a different flag, loopback GND->IO3
    // Test: set IO3 as input pullup. If it's low from the tied ground, we're wired in
    pinMode(IO3, INPUT_PULLUP);
    bool loopback_test_pass = false;

    // read IO3 as low 
    delay(1);
    if (!digitalRead(IO3)) loopback_test_pass = true;

    // if this works, GND->IO3 are bridged and debug connection is plugged in
    if (loopback_test_pass) {
        Serial2.println("Debug connection found. Please enter a command.");
        while (!Serial2.available()) {  // hang, waiting for a response
            delay(1);
        }
        char incomingByte = (char)Serial2.read();

        switch (incomingByte) {
            case 'R':
            case 'r':
                ReadAll();
                Serial2.println("> Read complete. Halting.");
                while (1);
                break;
            case 'E':
            case 'e':
                EraseEverything();
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

} // end namespace FlashLog