#pragma once
#include <cstdint>
#include <esp_err.h>
#include <array>

// mtch6301 conf
#define MTCH6301_I2C_ADDR 0x25
#define RESET_PIN 7              
#define INT_PIN 8                

#define SDA_PIN 5               
#define SCL_PIN 6

#define NUM_TX 3 // 3-13
#define NUM_RX 3 // 3-18

#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000    // 400kHz max for MTCH6301
#define I2C_MASTER_TIMEOUT_TI pdMS_TO_TICKS(10000) 

// Chip registers
#define MTCH6301_REG_INDEX_GEN 0x00
#define MTCH6301_REG_OFFSET_RX_CH 0x01
#define MTCH6301_REG_OFFSET_TX_CH 0x02

#define MTCH6301_REG_OFFSET_RX_COE_S 0x04
#define MTCH6301_REG_OFFSET_TX_COE_S 0x06

constexpr std::array<uint16_t, 16> scaling_coefficients = {
  0x5555,
  0x4000,
  0x3333,
  0x2AAA,
  0x2492,
  0x2000,
  0x1C71,
  0x1999,
  0x1745,
  0x1555,
  0x13B1,
  0x1249,
  0x1111,
  0x1000,
  0x0F0F,
  0x0E38
};

struct TouchPacket {
  uint8_t touchId;
  bool penDown;
  uint16_t x;
  uint16_t y;
};

void intHandler();
void performReset();
esp_err_t readTouchPacket(TouchPacket *pkt);
bool configureChip();
bool pingMTCH();
bool initI2C();

// MTCH Functions
esp_err_t mtch_write_register(uint8_t reg_index, uint8_t reg_offset, uint8_t value);
esp_err_t mtch_read_register(uint8_t reg_index, uint8_t reg_offset, uint8_t *res);
esp_err_t mtch_write_multi(uint8_t index, uint8_t offset_start, const uint8_t* data, size_t len);
esp_err_t mtch_read_multi(uint8_t index, uint8_t offset_start, uint8_t* res, size_t len);