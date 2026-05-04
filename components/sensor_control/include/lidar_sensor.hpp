#pragma once
#include <idf_c_includes.hpp>
#include "sensor_common.hpp"
#include "lydsto.hpp"

class LidarSensor {
public:
  explicit LidarSensor(const uart_port_t uart_port, int tx_pin, int rx_pin, uint32_t baud_rate);
  bool is_ready() const { return initialized_ && driver_.isReady(); }
  esp_err_t initialize();
  esp_err_t read(SensorCommon::LidarMeasurement& out);
private:
  LYDSTO_Driver driver_;
  bool initialized_;
  uart_port_t uart_port_;
  int tx_pin_;
  int rx_pin_;
  uint32_t baud_rate_;
};
