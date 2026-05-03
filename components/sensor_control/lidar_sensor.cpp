#include "lidar_sensor.hpp"
LidarSensor::LidarSensor(const uart_port_t uart_port, int tx_pin, int rx_pin, uint32_t baud_rate)
    : driver_(), initialized_(false) {
  (void)uart_port;
  (void)tx_pin;
  (void)rx_pin;
  (void)baud_rate;
}

esp_err_t LidarSensor::initialize() {
  const char* err = driver_.init();
  initialized_ = (err == nullptr);
  return initialized_ ? ESP_OK : ESP_FAIL;
}

esp_err_t LidarSensor::read(SensorCommon::LidarMeasurement& out) {
  if (!initialized_) return ESP_ERR_INVALID_STATE;
  LYDSTO_Driver::MeasurementResult m{};
  bool ok = driver_.read_sensor(m);
  out.active = true;
  out.valid = ok && m.valid;
  out.distance_mm = m.distance_mm;
  out.status = m.range_status;
  out.timestamp_us = m.timestamp_us;
  out.timeout_occurred = m.timeout_occurred;
  out.health = out.valid ? 1 : 2;
  return ok ? ESP_OK : ESP_FAIL;
}
