#include <Arduino.h>
#include <driver/i2c.h> // Had to use this insted of Wire.h cause it didn't work with clock streaching

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
#define I2C_MASTER_TIMEOUT_MS 10000 

// Chip registers
#define MTCH6301_REG_INDEX_GEN 0x00
#define MTCH6301_REG_OFFSET_RX_CH 0x01
#define MTCH6301_REG_OFFSET_TX_CH 0x02


volatile bool dataReady = false;
uint32_t count = 0;

void IRAM_ATTR intHandler();
void performReset();
void readTouchData();
bool configureChip();
bool pingMTCH();
bool initI2C();

// I2C Functions
esp_err_t i2c_master_read_slave(uint8_t *data_rd, size_t size);
esp_err_t i2c_master_write_read_slave(uint8_t *data_wr, size_t write_size, uint8_t *data_rd, size_t read_size);
esp_err_t i2c_write_register(uint8_t reg_index, uint8_t reg_offset, uint8_t value);
esp_err_t i2c_read_register(uint8_t reg_index, uint8_t reg_offset, uint8_t *res);

void IRAM_ATTR intHandler() {
  count++;
  dataReady = true;
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
}

void loop() {
  if (dataReady) {
    dataReady = false;
    Serial.print("INT triggered - ");
    Serial.println(count);
    // delay(5);
    readTouchData();
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

void readTouchData() {
  Serial.println("INT triggered - reading with ESP-IDF I2C...");
  
  uint8_t buffer[7];  
  
  esp_err_t ret = i2c_master_read_slave(buffer, sizeof(buffer));
  
  if (ret == ESP_OK) {
    uint8_t len = buffer[0];  // First byte is length
    Serial.print("Data length: ");
    Serial.println(len);
    
    if (len > 0 && len < sizeof(buffer)) {
      Serial.print("Data: ");
      for (int i = 1; i <= len; i++) {  
        Serial.print("0x");
        if (buffer[i] < 0x10) Serial.print("0");
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
  } else {
    Serial.printf("Read failed: %s\n", esp_err_to_name(ret));
  }
}

void performReset() {  
  digitalWrite(RESET_PIN, LOW);
  Serial.println("reset (low)");
  delay(500);  
  
  digitalWrite(RESET_PIN, HIGH);
  Serial.println("reset (high)");
  delay(1000);
}

esp_err_t i2c_master_read_slave(uint8_t *data_rd, size_t size) {
  return i2c_master_read_from_device(I2C_MASTER_PORT, MTCH6301_I2C_ADDR, 
                                    data_rd, size, 
                                    pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

esp_err_t i2c_master_write_read_slave(uint8_t *data_wr, size_t write_size, uint8_t *data_rd, size_t read_size) {
  return i2c_master_write_read_device(I2C_MASTER_PORT, MTCH6301_I2C_ADDR,
                                     data_wr, write_size,
                                     data_rd, read_size,
                                     pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

esp_err_t i2c_write_register(uint8_t index, uint8_t offset, uint8_t value) {
  uint8_t write_data[6];
  uint8_t resp[5];
  // Not sure what these to are
  write_data[0] = 0x55; //https://ww1.microchip.com/downloads/en/DeviceDoc/40001663B.pdf, Sync?
  write_data[1] = 0x4; // Size
  // Write register command
  write_data[2] = 0x15;
  // Data to write
  write_data[3] = index;
  write_data[4] = offset;
  write_data[5] = value;
  
  esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_PORT, MTCH6301_I2C_ADDR,
                                            write_data, 6,
                                            pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  
  if (ret == ESP_OK) {
    Serial.printf("Wrote 0x%02X to register 0x%01X%X\n", value, index, offset);
  } else {
    Serial.printf("Failed to write to register: %s\n",esp_err_to_name(ret));
  }

  // Read back
  vTaskDelay(pdMS_TO_TICKS(2));

  ret = i2c_master_read_from_device(I2C_MASTER_PORT, MTCH6301_I2C_ADDR, resp, sizeof(resp), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  return ret;
}

esp_err_t i2c_read_register(uint8_t index, uint8_t offset, uint8_t *res) {
    uint8_t cmd[5];
    uint8_t resp[6]; 

    cmd[0] = 0x55;
    cmd[1] = 0x03;
    cmd[2] = 0x16;
    cmd[3] = index;
    cmd[4] = offset;

    // Send command
    esp_err_t ret = i2c_master_write_to_device(
        I2C_MASTER_PORT,
        MTCH6301_I2C_ADDR,
        cmd,
        sizeof(cmd),
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
    );

    if (ret != ESP_OK) {
        Serial.printf("I2C write failed: %s\n", esp_err_to_name(ret));
        return ret;
    }

    Serial.println("Write done");
    vTaskDelay(pdMS_TO_TICKS(2));

    // Read response (7 bytes)
    ret = i2c_master_read_from_device(
        I2C_MASTER_PORT,
        MTCH6301_I2C_ADDR,
        resp,
        sizeof(resp),
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
    );

    if (ret == ESP_OK) {
        Serial.print("Raw response: ");
        for (int i = 0; i < sizeof(resp); i++) {
            Serial.printf("0x%02X ", resp[i]);
        }
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



