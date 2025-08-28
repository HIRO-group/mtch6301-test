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
#define I2C_MASTER_TIMEOUT_MS 5000 

// Chip registers
#define MTCH6301_REG_RX_CH 0x01
#define MTCH6301_REG_TX_CH 0x02


volatile bool dataReady = false;
uint8_t count = 0;

void IRAM_ATTR intHandler();
void performReset();
void readTouchData();
bool configureChip();
bool initI2C();

// I2C Functions
esp_err_t i2c_master_read_slave(uint8_t *data_rd, size_t size);
esp_err_t i2c_master_write_read_slave(uint8_t *data_wr, size_t write_size, uint8_t *data_rd, size_t read_size);
esp_err_t i2c_write_register(uint8_t reg_addr, uint8_t value);
esp_err_t i2c_read_register(uint8_t reg_addr, uint8_t *value);

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
  attachInterrupt(digitalPinToInterrupt(INT_PIN), intHandler, HIGH);

  delay(5000); // wait for console to be opened
  Serial.println("Starting");
  performReset();
  if (!initI2C()) {
    Serial.println("Failed to init I2C");
    while(1);
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
    readTouchData();
  }

  delay(1);
  
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
  if (i2c_write_register(MTCH6301_REG_RX_CH, NUM_RX) != ESP_OK) {
    Serial.println("Failed to write RX");
    return false;
  };

  if (i2c_write_register(MTCH6301_REG_TX_CH, NUM_TX) != ESP_OK) {
    Serial.println("Failed to write TX");
    return false;
  };

  uint8_t read_rx, read_tx;

   if (i2c_read_register(MTCH6301_REG_RX_CH, &read_rx) != ESP_OK) {
    Serial.println("Failed to verify RX channels");
    return false;
  }
  
  if (i2c_read_register(MTCH6301_REG_TX_CH, &read_tx) != ESP_OK) {
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

void readTouchData() {
  Serial.println("INT triggered - reading with ESP-IDF I2C...");
  
  uint8_t buffer[32];  //
  
  // Try to read touch data
  esp_err_t ret = i2c_master_read_slave(buffer, sizeof(buffer));
  
  if (ret == ESP_OK) {
    Serial.print("Successfully read data: ");
    
    // Find actual data length (stop at first 0xFF or after reasonable data)
    int dataLength = sizeof(buffer);
    for (int i = 0; i < sizeof(buffer); i++) {
      if (buffer[i] == 0xFF && i > 4) { // Assume data doesn't start with 0xFF
        dataLength = i;
        break;
      }
    }
    
    // Print hex data
    for (int i = 0; i < dataLength; i++) {
      Serial.print("0x");
      if (buffer[i] < 0x10) Serial.print("0");
      Serial.print(buffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
        
  } else if (ret == ESP_ERR_TIMEOUT) {
    Serial.println("Read timeout - device may be clock stretching extensively");
  } else {
    Serial.printf("Read failed with error: %s\n", esp_err_to_name(ret));
    
    // Try to recover the bus
    Serial.println("Attempting bus recovery...");
    i2c_driver_delete(I2C_MASTER_PORT);
    delay(100);
    initI2C();
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

esp_err_t i2c_write_register(uint8_t reg_addr, uint8_t value) {
  uint8_t write_data[2];
  write_data[0] = reg_addr;  // Register address
  write_data[1] = value;     // Value to write
  
  esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_PORT, MTCH6301_I2C_ADDR,
                                            write_data, 2,
                                            pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  
  if (ret == ESP_OK) {
    Serial.printf("Wrote 0x%02X to register 0x%02X\n", value, reg_addr);
  } else {
    Serial.printf("Failed to write to register 0x%02X: %s\n", reg_addr, esp_err_to_name(ret));
  }
  
  return ret;
}

esp_err_t i2c_read_register(uint8_t reg_addr, uint8_t *value) {
  // Write register address, then read the value
  esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_PORT, MTCH6301_I2C_ADDR,
                                              &reg_addr, 1,     // Write register address
                                              value, 1,         // Read 1 byte
                                              pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  
  if (ret == ESP_OK) {
    Serial.printf("Read 0x%02X from register 0x%02X\n", *value, reg_addr);
  } else {
    Serial.printf("Failed to read from register 0x%02X: %s\n", reg_addr, esp_err_to_name(ret));
  }
  
  return ret;
}

