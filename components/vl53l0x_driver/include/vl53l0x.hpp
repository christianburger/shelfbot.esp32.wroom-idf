#pragma once

#include <cstdint>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"

class VL53L0X {
public:
  // ===== CONSTRUCTOR =====
  VL53L0X(i2c_port_t i2c_port = I2C_NUM_0,
          gpio_num_t xshut_pin = GPIO_NUM_NC,
          uint8_t i2c_address = 0x29,
          bool io_2v8 = true);

  ~VL53L0X();

  // ===== SENSOR API =====
  const char* init();
  uint16_t readRangeSingleMillimeters();
  void setTimeout(uint16_t timeout_ms);
  bool timeoutOccurred() const;
  bool i2cFail() const;

  // Debug/Utility methods
  void dumpRegisters(uint8_t start = 0x00, uint8_t end = 0xFF);
  uint8_t getModuleType();

private:
  // Private methods
  uint8_t readReg(uint8_t reg);
  void writeReg(uint8_t reg, uint8_t value);
  uint16_t readReg16Bit(uint8_t reg);
  void writeReg16Bit(uint8_t reg, uint16_t value);
  bool readMulti(uint8_t reg, uint8_t* dest, uint8_t count);
  bool writeMulti(uint8_t reg, const uint8_t* src, uint8_t count);

  bool isConnected();
  uint8_t getModelID();

  // Configuration methods
  void setSignalRateLimit(float limit_Mcps);
  void setMeasurementTimingBudget(uint32_t budget_us);
  uint32_t getMeasurementTimingBudget();
  bool performRefCalibration(uint8_t vhv_init_byte);

  // Private member variables
  i2c_port_t i2c_port_;
  gpio_num_t xshut_pin_;
  uint8_t i2c_address_;
  bool io_2v8_;
  bool timeout_;
  bool i2c_fail_;
  uint16_t timeout_ms_;
  uint8_t stop_variable_;

  static const char* TAG;
};