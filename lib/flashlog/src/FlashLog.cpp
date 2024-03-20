#include <FlashLog.h>
#include "hardware/dma.h"
#include "hardware/structs/ssi.h"
#include "pico/stdlib.h"
#include "pico/time.h"
namespace FlashLog {

LogPage buf;  // One page buffer of bytes
constexpr size_t rx_chunk_size = 16;
LogPage buf_rx[rx_chunk_size];  // One page buffer of bytes
unsigned int current_flash_offset = FLASH_LOG_START;
unsigned int current_packet = 1;
unsigned int current_sector_packet = 0;

// based on https://github.com/MakerMatrix/RP2040_flash_programming/blob/main/RP2040_flash/RP2040_flash.ino
void FindFirstFreeSector() {
  int blocks_checked = 0;
  for (unsigned int i = FLASH_LOG_START; i < FLASH_LOG_END; i += FLASH_PAGE_SIZE) {
    blocks_checked++;
    if ( -1 == *((int *)(i)) ) {
      printf("Free block found at offset %d (%d blocks searched)\n", i-FLASH_LOG_START, blocks_checked);
      current_flash_offset = i;
      return;
    }
  }
  printf("Flash is full\n");
  current_flash_offset = PICO_FLASH_SIZE_BYTES;  // we're full
}


void __no_inline_not_in_flash_func(EraseEverything)() {
  int sector = 0;
  long begin = millis();
  for (unsigned int i = FLASH_LOG_START; i < FLASH_LOG_END; i += FLASH_SECTOR_SIZE) {
    long start = micros();
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(i-XIP_BASE, FLASH_SECTOR_SIZE);
    restore_interrupts (ints);
    printf("Erased sector %d/%d in %d us\n", sector, FLASH_LOG_SIZE/FLASH_SECTOR_SIZE, micros() - start);
    sector += 1;
  }
  current_flash_offset = FLASH_LOG_START;
  printf("Erased flash in %dms\n", millis()-begin);
}

LogEntry* __not_in_flash_func(GetNextEntry)() {
  if (current_sector_packet == 0) {  // first packet
    buf.packet = current_packet;
    // printf("Starting fresh FlashLog page %d\n", buf.packet);
  }

  

  if (current_sector_packet < packets_per_block) {
    return &buf.data[current_sector_packet++];
  } else {
    return nullptr;
  }
}

void __not_in_flash_func(WritePacketIfFull)() {

  if (current_flash_offset >= FLASH_LOG_END) {
    Serial2.println("Flash full");
    return;
  }

  //printf("Copying packet in to %d\n", current_sector_packet);
  // memcpy(&buf.data[current_sector_packet], &packet, sizeof(LogEntry));

  // if full, write out now!
  if (current_sector_packet == packets_per_block) {
    //Serial2.printf("FlashLog wrote page %d\n", buf.packet);
    //Serial2.printf("Filled FlashLog page %d, writing to offset %d\n", buf.packet, current_flash_offset - XIP_BASE);
    auto start = micros();
    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(current_flash_offset-XIP_BASE, (uint8_t *)(&buf), FLASH_PAGE_SIZE);
    restore_interrupts (ints);
    Serial2.printf("Done writing %u bytes in %d us\n", FLASH_PAGE_SIZE, micros()-start);
    current_flash_offset += FLASH_PAGE_SIZE;
    current_sector_packet = 0;
    current_packet++;
  }
}

void __no_inline_not_in_flash_func(WriteBasic)(uint16_t battery_mV, bool radio_connected, const std::array<uint16_t, 6>& rc, const std::array<uint16_t, 4> out) {
    LogEntry *entry = GetNextEntry();
    if (entry == nullptr) return;
    
    entry->packetType = 1;  // todo: Enum!
    entry->timestamp = (uint32_t)micros();  // new format: just use microseconds

    BasicLog1& bl1 = entry->basic_log_1;
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

    WritePacketIfFull();
}

void __no_inline_not_in_flash_func(WriteESC)(uint64_t timestamp_us, uint8_t esc, 
                      uint32_t rpm, uint8_t temperature_C, 
                      uint16_t volts_cV, uint8_t amps_A) {
    LogEntry *entry = GetNextEntry();
    if (entry == nullptr) return;

    entry->packetType = 2; // todo: Enum!
    entry->timestamp = (uint32_t)(timestamp_us);  // new format: just use microseconds

    ESCLog2& el2 = entry->esc_log_2;

    el2.esc = esc & BITMASK_4;  // output restricted to 4 bits
    el2.rpm = rpm;
    el2.temperature_C = temperature_C;
    el2.volts_cV = volts_cV;
    el2.amps_A = amps_A;

    WritePacketIfFull();
}

void __no_inline_not_in_flash_func(WriteIMU)(const std::array<int16_t, 3>& gyro, const std::array<int16_t, 3>& accel) {
    LogEntry *entry = GetNextEntry();
    if (entry == nullptr) return;
    
    entry->packetType = 3;  // todo: Enum!
    entry->timestamp = (uint32_t)micros();  // new format: just use microseconds

    IMULog3& il3 = entry->imu_log_3;
    il3.gyro[0] = gyro[0];
    il3.gyro[1] = gyro[1];
    il3.gyro[2] = gyro[2];
    il3.accel[0] = accel[0];
    il3.accel[1] = accel[1];
    il3.accel[2] = accel[2];

    WritePacketIfFull();
}

void __not_in_flash_func(PrintDecimal8Bit)(uint32_t val) {
   printf("%d.%04d ", val>>8, (uint16_t)((uint64_t)(val & 0b11111111)*1000/256));
}
void __not_in_flash_func(PrintDecimal4Bit)(uint16_t val) {
  printf("%d.%03d ", val>>4, 625*(uint16_t)(val & 0b1111));
}

void __no_inline_not_in_flash_func(PrintPacket)(uint32_t page, uint8_t packet, LogEntry& entry) {
    printf("%d %d %d %u ", page, packet, entry.packetType, entry.timestamp);
    switch (entry.packetType) {
        case 1:  // BasicLog1
            {
              BasicLog1& bl1 = entry.basic_log_1;
              PrintDecimal4Bit(bl1.battery);
              printf("%u %u %u %u %u %u %u ", bl1.radio_connected, bl1.rc_ch1, bl1.rc_ch2, bl1.rc_ch3, bl1.rc_ch4 << 7, bl1.rc_ch5 << 7, bl1.rc_ch6 << 7);
              printf("%u %u %u %u\n", bl1.out0, bl1.out1, bl1.out2, bl1.out3);
            }
            break;
        case 2: // ESCLog2
          {
            ESCLog2& el2 = entry.esc_log_2;

            printf("%u %u %u %u %d\n", el2.esc, el2.rpm, el2.temperature_C, el2.volts_cV, el2.amps_A);
          }
            break;
        case 3: // IMULog3
          {
            IMULog3& il3 = entry.imu_log_3;
            printf("%d %d %d %d %d %d\n", il3.gyro[0], il3.gyro[1], il3.gyro[2], il3.accel[0], il3.accel[1], il3.accel[2]);
          }
            break;
        default:
            Serial2.println("Unknown packet type");
            break;
    }
}

// from https://github.com/raspberrypi/pico-examples/blob/master/flash/ssi_dma/flash_ssi_dma.c
void __no_inline_not_in_flash_func(flash_bulk_read)(uint32_t *rxbuf, uint32_t flash_offs, size_t len,
                                                 uint dma_chan) {
    // SSI must be disabled to set transfer size. If software is executing
    // from flash right now then it's about to have a bad time
    ssi_hw->ssienr = 0;
    ssi_hw->ctrlr1 = len - 1; // NDF, number of data frames
    ssi_hw->dmacr = SSI_DMACR_TDMAE_BITS | SSI_DMACR_RDMAE_BITS;
    ssi_hw->ssienr = 1;
    // Other than NDF, the SSI configuration used for XIP is suitable for a bulk read too.

    // Configure and start the DMA. Note we are avoiding the dma_*() functions
    // as we can't guarantee they'll be inlined
    dma_hw->ch[dma_chan].read_addr = (uint32_t) &ssi_hw->dr0;
    dma_hw->ch[dma_chan].write_addr = (uint32_t) rxbuf;
    dma_hw->ch[dma_chan].transfer_count = len;
    // Must enable DMA byteswap because non-XIP 32-bit flash transfers are
    // big-endian on SSI (we added a hardware tweak to make XIP sensible)
    dma_hw->ch[dma_chan].ctrl_trig =
            DMA_CH0_CTRL_TRIG_BSWAP_BITS |
            DREQ_XIP_SSIRX << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB |
            dma_chan << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB |
            DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS |
            DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB |
            DMA_CH0_CTRL_TRIG_EN_BITS;

    // Now DMA is waiting, kick off the SSI transfer (mode continuation bits in LSBs)
    ssi_hw->dr0 = (flash_offs << 8u) | 0xa0u;

    // Wait for DMA finish
    while (dma_hw->ch[dma_chan].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS);

    // Reconfigure SSI before we jump back into flash!
    ssi_hw->ssienr = 0;
    ssi_hw->ctrlr1 = 0; // Single 32-bit data frame per transfer
    ssi_hw->dmacr = 0;
    ssi_hw->ssienr = 1;
}

void __not_in_flash_func(ReadAll)() {
  LogPage *p;
  Serial2.println("> Read start");
  uint32_t page = 0;
  auto start = micros();

  for (unsigned int i = FLASH_LOG_START; i < FLASH_LOG_END; i += FLASH_PAGE_SIZE*rx_chunk_size) {

    // fetch data from flash, bypassing the XIP cache.
    p = (LogPage *)(i+(XIP_NOCACHE_NOALLOC_BASE - XIP_BASE));
    memcpy(&buf_rx, p, FLASH_PAGE_SIZE*rx_chunk_size);  // read the whole page to buf_rx
    // flash_bulk_read((uint32_t*)buf_rx, i, FLASH_PAGE_SIZE/sizeof(uint32_t)*rx_chunk_size, 0);

    printf("> Page %d\n", buf_rx[0].packet);
    for(uint32_t k = 0; k<rx_chunk_size; k++) {
      if (buf_rx[k].packet == -1) {
        i = FLASH_LOG_END;  // exit i loop too
        break;
      }
      for (unsigned int j=0; j<packets_per_block; j++) {
        PrintPacket(page+k, j, buf_rx[k].data[j]);
      }
    }

    page+=rx_chunk_size;
  }
  auto duration_us = micros()-start;
  printf("> Read %d flashlog entries in %luus, %dkB/s\n", page, duration_us, FLASH_PAGE_SIZE*page*1000/duration_us);
}


void Setup(bool skip_prompt) {
    
    
    Serial2.print("Flash init, max storage "); Serial2.print(FLASH_LOG_SIZE / 1024); Serial2.println("kB");
    Serial2.println("FLASH_PAGE_SIZE = " + String(FLASH_PAGE_SIZE, DEC));
    Serial2.println("FLASH_SECTOR_SIZE = " + String(FLASH_SECTOR_SIZE, DEC));
    Serial2.println("FLASH_BLOCK_SIZE = " + String(FLASH_BLOCK_SIZE, DEC));
    Serial2.println("PICO_FLASH_SIZE_BYTES = " + String(PICO_FLASH_SIZE_BYTES, DEC));
    printf("FlashLog Max Entries = %d\n", FLASH_LOG_SIZE/FLASH_PAGE_SIZE*packets_per_block);
    Serial2.println("XIP_BASE = 0x" + String(XIP_BASE, HEX));
    printf("Page buffer size: %d\n", sizeof(LogPage));

    FindFirstFreeSector();
    

    if (!skip_prompt) {
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
}

} // end namespace FlashLog