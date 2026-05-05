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
  bool get_last_raw_packet(uint8_t* out, size_t len) const { return driver_.get_last_packet(out, len); }
  uint32_t packet_count() const { return driver_.get_packet_count(); }
private:
  LYDSTO_Driver driver_;
  bool initialized_;
  uart_port_t uart_port_;
  int tx_pin_;
  int rx_pin_;
  uint32_t baud_rate_;
};
