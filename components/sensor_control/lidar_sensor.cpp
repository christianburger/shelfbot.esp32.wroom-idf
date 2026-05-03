#include "lidar_sensor.hpp"
esp_err_t LidarSensor::read(SensorCommon::LidarMeasurement& out) {
  if (!tof_) return ESP_ERR_INVALID_STATE;
  SensorCommon::TofMeasurement m{};
  esp_err_t e = tof_->read_sensor(0, m);
  out.active = true;
  out.valid = (e == ESP_OK) && m.valid;
  out.distance_mm = m.distance_mm;
  out.status = m.status;
  out.timestamp_us = m.timestamp_us;
  out.timeout_occurred = m.timeout_occurred;
  out.health = out.valid ? 1 : 2;
  return e;
}
