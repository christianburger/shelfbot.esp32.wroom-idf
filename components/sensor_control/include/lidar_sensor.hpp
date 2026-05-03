#pragma once
#include <idf_c_includes.hpp>
#include "sensor_common.hpp"
#include "tof_sensor.hpp"

class LidarSensor {
public:
  struct Config {
    bool enabled = true;
  };

  LidarSensor(TofSensor* tof_sensor, const Config& config);
  ~LidarSensor() = default;

  esp_err_t initialize();
  bool is_ready() const;
  esp_err_t read(SensorCommon::LidarMeasurement& measurement);

private:
  TofSensor* tof_sensor_;
  Config config_;
  bool initialized_;
};
