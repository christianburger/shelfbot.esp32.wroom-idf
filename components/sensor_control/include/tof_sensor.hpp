#pragma once
#include <idf_c_includes.hpp>
#include "sensor_common.hpp"

// Forward declaration - no driver header
class TofDriver;

class TofSensor {
public:
  struct Config {
    struct TofConfig {
      uint8_t device_id = 0;
      uint32_t timeout_ms = 500;
      bool enabled = true;
    };
    TofConfig tof_configs[SensorCommon::NUM_TOF_SENSORS];
    uint32_t tof_read_interval_ms = 200;
    std::function<void(const SensorCommon::TofMeasurement*)> tof_callback = nullptr;
  };

  TofSensor(const Config& config);
  ~TofSensor();

  esp_err_t initialize();
  bool is_ready() const;
  esp_err_t read_all(SensorCommon::TofMeasurement results[SensorCommon::NUM_TOF_SENSORS]);
  esp_err_t read_sensor(uint8_t sensor_index, SensorCommon::TofMeasurement& result);
  esp_err_t start_continuous();
  esp_err_t stop_continuous();
  bool is_continuous() const;
  esp_err_t self_test(uint8_t sensor_index);
  bool probe(uint8_t sensor_index);
  bool is_sensor_ready(uint8_t sensor_index) const;

private:
  Config config_;
  bool initialized_;
  bool continuous_mode_;

  TofDriver* drivers_[SensorCommon::NUM_TOF_SENSORS];  // Opaque pointers
  bool sensor_enabled_[SensorCommon::NUM_TOF_SENSORS];

  SensorCommon::TofMeasurement last_measurements_[SensorCommon::NUM_TOF_SENSORS];
  int64_t last_read_time_us_;

  esp_err_t initialize_driver(uint8_t sensor_index);
  esp_err_t destroy_driver(uint8_t sensor_index);

  static const char* TAG;

  TofSensor(const TofSensor&) = delete;
  TofSensor& operator=(const TofSensor&) = delete;
};
