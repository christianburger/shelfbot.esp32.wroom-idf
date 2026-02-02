// [file name]: sensor_manager.hpp
#pragma once
#include "sensor_common.hpp"
#include "sensor_control.hpp"
#include <idf_c_includes.hpp>

class SensorManager {
public:
  static SensorManager& get_instance() {
    static SensorManager instance;
    return instance;
  }

  // Initialize with configuration
  void initialize(const SensorControl::Config& config);

  // Start continuous reading
  void start();

  // Stop continuous reading
  void stop();

  // Get the latest sensor data
  bool get_latest_data(SensorCommon::SensorDataPacket& data);

  // Get sensor control instance (for direct control if needed)
  SensorControl* get_sensor_control() { return sensor_control_.get(); }

  // Check if initialized
  bool is_initialized() const { return initialized_; }

private:
  SensorManager() = default;
  ~SensorManager() = default;

  SensorManager(const SensorManager&) = delete;
  SensorManager& operator=(const SensorManager&) = delete;

  std::unique_ptr<SensorControl> sensor_control_;
  SensorCommon::SensorDataPacket latest_data_;
  SemaphoreHandle_t data_mutex_;
  bool initialized_ = false;
};