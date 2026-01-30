#pragma once

#include "sensor_common.hpp"
#include "tof400f_modbus.hpp"
#include <memory>

/**
 * @brief TOF400F sensor using MODBUS exclusively (no I2C)
 *
 * This driver uses UART/MODBUS for all operations:
 * - Configuration via MODBUS registers
 * - Distance reading via register 0x0010
 * - No I2C required
 */
class Tof400fModbusSensor {
public:
  struct Config {
    // UART/MODBUS configuration
    uart_port_t uart_port;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    uint32_t baud_rate;
    uint8_t modbus_address;

    // Sensor configuration
    Tof400fModbus::RangingMode ranging_mode;
    uint16_t timeout_ms;
  };

  explicit Tof400fModbusSensor(const Config& config);
  ~Tof400fModbusSensor() = default;

  /**
   * @brief Initialize sensor via MODBUS
   * @return nullptr on success, error string on failure
   */
  const char* init();

  /**
   * @brief Check if sensor is ready
   */
  bool isReady() const;

  /**
   * @brief Read distance measurement via MODBUS
   * @param result Output reading
   * @return true on success
   */
  bool readSingle(SensorCommon::Reading& result);

  /**
   * @brief Set ranging mode via MODBUS
   */
  const char* setRangingMode(Tof400fModbus::RangingMode mode);

  /**
   * @brief Perform self-test
   */
  const char* selfTest();

private:
  Config config_;
  std::unique_ptr<Tof400fModbus> modbus_;
  bool initialized_;

  static const char* TAG;
  static constexpr uint16_t MEASUREMENT_RESULT_REGISTER = 0x0010;
};
