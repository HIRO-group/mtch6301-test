#include <Arduino.h>
#include <driver/i2c.h> // Had to use this insted of Wire.h cause it didn't work with clock streaching

// mtch6301 conf
#define MTCH6301_I2C_ADDR 0x25
#define RESET_PIN 7              
#define INT_PIN 8                

#define SDA_PIN 5               
#define SCL_PIN 6

#define NUM_TX 7 // 3-13
#define NUM_RX 3 // 3-18

#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000    // 400kHz max for MTCH6301
#define I2C_MASTER_TIMEOUT_TI pdMS_TO_TICKS(10000) 

// Chip registers
#define MTCH6301_REG_INDEX_GEN 0x00
#define MTCH6301_REG_OFFSET_RX_CH 0x01
#define MTCH6301_REG_OFFSET_TX_CH 0x02

struct TouchPacket {
  uint8_t touchId;
  bool penDown;
  uint16_t x;
  uint16_t y;
};

volatile bool dataReady = false;
volatile bool ready = false;
uint32_t count = 0;

void IRAM_ATTR intHandler();
void performReset();
esp_err_t readTouchPacket();
bool configureChip();
bool pingMTCH();
bool initI2C();

// I2C Functions
esp_err_t i2c_write_register(uint8_t reg_index, uint8_t reg_offset, uint8_t value);
esp_err_t i2c_read_register(uint8_t reg_index, uint8_t reg_offset, uint8_t *res);

void IRAM_ATTR intHandler() {
  count++;
  if (ready) dataReady = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  pinMode(RESET_PIN, OUTPUT);
  pinMode(INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), intHandler, RISING);

  delay(5000); // wait for console to be opened
  Serial.println("Starting");
  performReset();
  if (!initI2C()) {
    Serial.println("Failed to init I2C");
    while(1);
  }

  if(!pingMTCH()) {
    Serial.println("Failed to ping chip");
    while (1);
  }

  if (!configureChip()) {
    Serial.println("Failed to config chip");
    while (1);    
  }

  Serial.println("Init done");
  ready = true;
}

void loop() {
  if (dataReady) {
    dataReady = false;
    TouchPacket pkt;
    esp_err_t res = readTouchPacket(&pkt);
    if (res == ESP_OK) {
      Serial.write(pkt.touchId);
      Serial.write(pkt.penDown ? 1 : 0);
      Serial.write(pkt.x & 0xFF);
      Serial.write((pkt.x >> 8) & 0xFF);
      Serial.write(pkt.y & 0xFF);
      Serial.write((pkt.y >> 8) & 0xFF);
    } else {
      Serial.print("Error reading touch data ");
      Serial.println(esp_err_to_name(res));
    }
  }
}

bool initI2C() {
  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = (gpio_num_t)SDA_PIN;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = (gpio_num_t)SCL_PIN;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;  // Use normal clock source
  
  // Configure and install I2C driver
  esp_err_t err = i2c_param_config(I2C_MASTER_PORT, &conf);
  if (err != ESP_OK) {
    Serial.printf("I2C param config failed: %s\n", esp_err_to_name(err));
    return false;
  }
  
  err = i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
  if (err != ESP_OK) {
    Serial.printf("I2C driver install failed: %s\n", esp_err_to_name(err));
    return false;
  }
  
  
  Serial.println("ESP-IDF I2C driver initialized successfully");
  return true;
}

bool configureChip() {
  // write values
  if (i2c_write_register(MTCH6301_REG_INDEX_GEN, MTCH6301_REG_OFFSET_RX_CH, NUM_RX) != ESP_OK) {
    Serial.println("Failed to write RX");
    return false;
  };

  if (i2c_write_register(MTCH6301_REG_INDEX_GEN, MTCH6301_REG_OFFSET_TX_CH, NUM_TX) != ESP_OK) {
    Serial.println("Failed to write TX");
    return false;
  };

  uint8_t read_rx, read_tx;
  delay(12);

   if (i2c_read_register(MTCH6301_REG_INDEX_GEN, MTCH6301_REG_OFFSET_RX_CH, &read_rx) != ESP_OK) {
    Serial.println("Failed to verify RX channels");
    return false;
  }
  
  if (i2c_read_register(MTCH6301_REG_INDEX_GEN, MTCH6301_REG_OFFSET_TX_CH, &read_tx) != ESP_OK) {
    Serial.println("Failed to verify TX channels");
    return false;
  }

  if (read_rx == NUM_RX && read_tx == NUM_TX) {
    Serial.printf("Configuration verified: RX=%d, TX=%d\n", read_rx, read_tx);
    return true;
  } else {
    Serial.printf("Configuration mismatch: Expected RX=%d TX=%d, Got RX=%d TX=%d\n", 
                  NUM_RX, NUM_TX, read_rx, read_tx);
    return false;
  }

}

bool pingMTCH() {
  uint8_t test;
  esp_err_t ret = i2c_master_read_from_device(I2C_MASTER_PORT, MTCH6301_I2C_ADDR, &test, 1, pdMS_TO_TICKS(100));
  if (ret == ESP_OK) {
    Serial.println("Device responded");
    return true;
  } else {
    Serial.printf("MTCH not responding: %s\n", esp_err_to_name(ret));
    return false;
  }
}

esp_err_t readTouchPacket(TouchPacket* pkt) {
  uint8_t buffer[6];  
  esp_err_t ret = i2c_master_read_from_device(
      I2C_MASTER_PORT, MTCH6301_I2C_ADDR, buffer, sizeof(buffer),
      I2C_MASTER_TIMEOUT_TI
  );

  if (ret != ESP_OK) return ret;

  // buffer[0] is the length (always 0x05), ignore it
  // The actual touch data starts at buffer[1] (D0-D4)
  
  // D0 = buffer[1]: Extract TOUCHID<3:0> (bits 6-3) and PEN (bit 0)
  pkt->touchId = (buffer[1] >> 3) & 0x0F;   // Extract bits 6-3
  pkt->penDown = buffer[1] & 0x01;          // Extract bit 0

  // D1 = buffer[2], D2 = buffer[3]: Extract X coordinate
  // D1 contains X<6:0> in bits 6-0
  // D2 contains X<11:7> in bits 4-0
  uint16_t x_low = buffer[2] & 0x7F;        // X<6:0> from D1
  uint16_t x_high = buffer[3] & 0x1F;       // X<11:7> from D2
  pkt->x = (x_high << 7) | x_low;            // Combine: X<11:7> << 7 | X<6:0>

  // D3 = buffer[4], D4 = buffer[5]: Extract Y coordinate
  // D3 contains Y<6:0> in bits 6-0
  // D4 contains Y<11:7> in bits 4-0
  uint16_t y_low = buffer[4] & 0x7F;        // Y<6:0> from D3
  uint16_t y_high = buffer[5] & 0x1F;       // Y<11:7> from D4
  pkt->y = (y_high << 7) | y_low;            // Combine: Y<11:7> << 7 | Y<6:0>

  return ESP_OK;
}

void performReset() {  
  digitalWrite(RESET_PIN, LOW);
  Serial.println("reset (low)");
  delay(500);  
  
  digitalWrite(RESET_PIN, HIGH);
  Serial.println("reset (high)");
  delay(1000);
}

esp_err_t i2c_write_register(uint8_t index, uint8_t offset, uint8_t value) {
  // https://ww1.microchip.com/downloads/en/DeviceDoc/40001663B.pdf
  uint8_t write_data[6] = {0x55, 0x4, 0x15, index, offset, value};
  uint8_t resp[5];
  
  esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_PORT, MTCH6301_I2C_ADDR,
                                            write_data, 6,
                                            I2C_MASTER_TIMEOUT_TI);
  if (ret == ESP_OK) {
    Serial.printf("Wrote 0x%02X to register 0x%01X%X\n", value, index, offset);
  } else {
    Serial.printf("Failed to write to register: %s\n",esp_err_to_name(ret));
  }

  // Read back
  vTaskDelay(pdMS_TO_TICKS(2));

  ret = i2c_master_read_from_device(I2C_MASTER_PORT, MTCH6301_I2C_ADDR, resp, sizeof(resp), I2C_MASTER_TIMEOUT_TI);
  return ret;
}

esp_err_t i2c_read_register(uint8_t index, uint8_t offset, uint8_t *res) {
  // https://ww1.microchip.com/downloads/en/DeviceDoc/40001663B.pdf
  uint8_t cmd[5] = {0x55, 0x03, 0x16, index, offset};
  uint8_t resp[6]; 

  // Send command
  esp_err_t ret = i2c_master_write_to_device(
      I2C_MASTER_PORT,
      MTCH6301_I2C_ADDR,
      cmd,
      sizeof(cmd),
      I2C_MASTER_TIMEOUT_TI
  );

  if (ret != ESP_OK) {
      Serial.printf("I2C write failed: %s\n", esp_err_to_name(ret));
      return ret;
  }

  vTaskDelay(pdMS_TO_TICKS(2));

  // Read response (7 bytes)
  ret = i2c_master_read_from_device(
      I2C_MASTER_PORT,
      MTCH6301_I2C_ADDR,
      resp,
      sizeof(resp),
      I2C_MASTER_TIMEOUT_TI
  );

  if (ret == ESP_OK) {
      Serial.println();

      if (resp[1] == 0x55 && resp[2] == 0x03 && resp[3] == 0x00) {
          *res = resp[5];
          Serial.printf("Read 0x%02X from register index=0x%02X offset=0x%02X\n",
                        *res, index, offset);
      } else {
          Serial.printf("Invalid response format!\n");
          ret = ESP_FAIL;
      }
  } else {
      Serial.printf("I2C read failed: %s\n", esp_err_to_name(ret));
  }

  return ret;
}



