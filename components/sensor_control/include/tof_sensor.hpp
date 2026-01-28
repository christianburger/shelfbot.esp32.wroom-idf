#pragma once

#include <vector>
#include <memory>
#include "vl53l0x.hpp"
#include "../../vl53l1_driver/include/vl53l1.hpp"
#include "sensor_common.hpp"
// ... rest of includes ...

struct ToFSensorConfig {
  i2c_port_t i2c_port = I2C_NUM_0;
  uint8_t i2c_address = 0x29;
  gpio_num_t sda_pin = GPIO_NUM_21;
  gpio_num_t scl_pin = GPIO_NUM_22;
  gpio_num_t xshut_pin = GPIO_NUM_NC;

  // Add VL53L1 specific fields
  bool use_vl53l1 = false;              // New: Select sensor type
  uart_port_t uart_port = UART_NUM_1;   // New: For VL53L1 mode switch
  gpio_num_t uart_tx_pin = GPIO_NUM_17; // New: UART TX
  gpio_num_t uart_rx_pin = GPIO_NUM_16; // New: UART RX
  VL53L1::RangingMode ranging_mode =
      VL53L1::RangingMode::HIGH_PRECISION; // New: VL53L1 mode

  bool io_2v8 = true;
  uint16_t timeout_ms = 500;
  uint32_t timing_budget_us = 200000;
  float signal_rate_limit_mcps = 0.25;
};

class ToFSensorArray {
public:
  explicit ToFSensorArray(uint8_t num_sensors);
  ~ToFSensorArray() = default;

  bool add_sensor(uint8_t index, const ToFSensorConfig& cfg);
  bool update_readings(std::vector<SensorCommon::Reading>& readings);

private:
  std::vector<std::unique_ptr<VL53L0X>> vl53l0x_sensors_;
  std::vector<std::unique_ptr<VL53L1>> vl53l1_sensors_;  // Add this
  std::vector<bool> is_vl53l1_;  // Track which type each sensor is
  uint8_t num_sensors_;
};