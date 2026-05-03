#pragma once
#include <idf_c_includes.hpp>
#include "sensor_common.hpp"
#include "tof_sensor.hpp"

class LidarSensor {
public:
  explicit LidarSensor(TofSensor* tof) : tof_(tof) {}
  bool is_ready() const { return tof_ && tof_->is_sensor_ready(0); }
  esp_err_t read(SensorCommon::LidarMeasurement& out);
private:
  TofSensor* tof_;
};
