#pragma once

#include <cstdint>
#include <memory>
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "idf_c_includes.hpp"

class VL53L1 {
public:
  enum class RangingMode : uint8_t {
    HIGH_PRECISION,   // Short range (~1.3m)
    LONG_DISTANCE     // Long range (~4.0m)
  };

  enum class MeasurementMode : uint8_t {
    SINGLE,
    CONTINUOUS
  };

  struct Config {
    i2c_port_t   i2c_port;
    gpio_num_t   sda_pin;
    gpio_num_t   scl_pin;
    uint8_t      i2c_address;
    uint32_t     i2c_freq_hz;
    RangingMode  ranging_mode;
    uint16_t     timeout_ms;
  };

  struct MeasurementResult {
    int64_t  timestamp_us;
    uint16_t distance_mm;
    uint8_t  range_status;
    bool     valid;
    bool     timeout_occurred;
  };

  explicit VL53L1(const Config& config);
  ~VL53L1();

  VL53L1(VL53L1&& other) noexcept;
  VL53L1& operator=(VL53L1&& other) noexcept;
  VL53L1(const VL53L1&) = delete;
  VL53L1& operator=(const VL53L1&) = delete;

  const char* init();
  bool isReady() const;
  bool readSingle(MeasurementResult& result);
  bool startContinuous();
  bool readContinuous(MeasurementResult& result);
  bool stopContinuous();
  const char* setRangingMode(RangingMode mode);
  RangingMode getRangingMode() const;
  void setTimeout(uint16_t timeout_ms);
  uint16_t getTimeout() const;
  bool setAddress(uint8_t new_addr);
  uint8_t getAddress() const;
  bool timeoutOccurred();
  bool probe();
  const char* selfTest();

private:
  struct Impl;
  std::unique_ptr<Impl> pimpl_;
};

VL53L1::Config vl53l1_default_config(
    i2c_port_t port = I2C_NUM_0,
    gpio_num_t sda = GPIO_NUM_21,
    gpio_num_t scl = GPIO_NUM_22
);