#pragma once

#include <vector>
#include <memory>
#include "vl53l0x.hpp"
#include "sensor_common.hpp"
#include "driver/gpio.h"
#include "driver/i2c.h"

// Configuration for a single ToF sensor - Updated for new driver interface
struct ToFSensorConfig {
  i2c_port_t i2c_port = I2C_NUM_0;      // I2C port number
  uint8_t i2c_address = 0x29;           // I2C device address
  gpio_num_t sda_pin = GPIO_NUM_21;     // SDA pin (required for I2C init)
  gpio_num_t scl_pin = GPIO_NUM_22;     // SCL pin (required for I2C init)
  gpio_num_t xshut_pin = GPIO_NUM_NC;   // XSHUT pin (optional)
  bool io_2v8 = true;                   // IO voltage (true = 2.8V, false = 1.8V)
};

// Low-level array manager (talks to driver)
class ToFSensorArray {
public:
  explicit ToFSensorArray(uint8_t num_sensors);

  bool add_sensor(uint8_t index, const ToFSensorConfig& cfg);
  bool update_readings(std::vector<SensorCommon::Reading>& readings);

private:
  std::vector<std::shared_ptr<VL53L0X>> sensors_;
  uint8_t num_sensors_;
  bool i2c_initialized_ = false;
};

// High-level manager with task and queue
class ToFSensorManager {
public:
  static ToFSensorManager& instance();

  bool configure(const ToFSensorConfig* sensor_configs, uint8_t num_sensors);
  bool start_reading_task(uint32_t read_interval_ms, UBaseType_t priority);
  bool get_latest_readings(std::vector<SensorCommon::Reading>& readings);

  void pause();
  void resume();
  void stop();

private:
  ToFSensorManager();
  ~ToFSensorManager();

  static void reading_task(void* param);

  struct TaskParams {
    ToFSensorManager* manager;
    uint32_t interval_ms;
  };

  ToFSensorArray* array_;
  std::vector<SensorCommon::Reading> latest_readings_;
  SemaphoreHandle_t data_mutex_;
  TaskHandle_t task_handle_;
  bool running_;
  bool paused_;
};
