#include <Arduino.h>
#include <driver/i2c.h>

// mtch6301 conf
#define MTCH6301_I2C_ADDR 0x25
#define RESET_PIN 7              
#define INT_PIN 8                

#define SDA_PIN 5               
#define SCL_PIN 6

#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000    // 400kHz max for MTCH6301
#define I2C_MASTER_TIMEOUT_MS 5000 


volatile bool dataReady = false;
uint8_t count = 0;

void IRAM_ATTR intHandler();
void performReset();
void readTouchData();
bool pingChip();
esp_err_t i2c_master_read_slave(uint8_t *data_rd, size_t size);
esp_err_t i2c_master_write_read_slave(uint8_t *data_wr, size_t write_size, uint8_t *data_rd, size_t read_size);
bool initI2C();

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

bool pingChip() {
  Serial.println("Pinging chip");
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
