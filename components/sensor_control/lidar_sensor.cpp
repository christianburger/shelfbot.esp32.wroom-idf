#include "lidar_sensor.hpp"

LidarSensor::LidarSensor(TofSensor* tof_sensor, const Config& config)
    : tof_sensor_(tof_sensor), config_(config), initialized_(false) {}

esp_err_t LidarSensor::initialize() {
  initialized_ = (config_.enabled && tof_sensor_ != nullptr);
  return initialized_ ? ESP_OK : ESP_FAIL;
}

bool LidarSensor::is_ready() const {
  return initialized_ && tof_sensor_ && tof_sensor_->is_sensor_ready(0);
}

esp_err_t LidarSensor::read(SensorCommon::LidarMeasurement& measurement) {
  if (!initialized_ || !tof_sensor_) return ESP_ERR_INVALID_STATE;

  SensorCommon::TofMeasurement tof{};
  esp_err_t err = tof_sensor_->read_sensor(0, tof);
  if (err != ESP_OK) return err;

  measurement.distance_mm = tof.distance_mm;
  measurement.valid = tof.valid;
  measurement.status = tof.status;
  measurement.timestamp_us = tof.timestamp_us;
  measurement.timeout_occurred = tof.timeout_occurred;
  return ESP_OK;
}
