#pragma once

#include <vector>
#include <memory>
#include "vl53l0x.hpp"
#include "../../vl53l1_driver/include/vl53l1.hpp"
#include "sensor_common.hpp"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// Forward declarations
class ToFSensorArray;
class ToFSensorManager;

/**
 * @brief Configuration for a single ToF sensor slot
 *
 * The system will automatically detect which sensor type is present
 * (VL53L0X or VL53L1) via I2C scanning after attempting UART mode switch.
 */
struct ToFSensorConfig {
  i2c_port_t i2c_port = I2C_NUM_0;
  uint8_t i2c_address = 0x29;
  gpio_num_t sda_pin = GPIO_NUM_21;
  gpio_num_t scl_pin = GPIO_NUM_22;
  gpio_num_t xshut_pin = GPIO_NUM_NC;

  // UART configuration for VL53L1 TOF400F mode switching
  // These are used for the UART->I2C mode switch workaround
  uart_port_t uart_port = UART_NUM_1;
  gpio_num_t uart_tx_pin = GPIO_NUM_17;
  gpio_num_t uart_rx_pin = GPIO_NUM_16;

  // VL53L1 specific settings (used if VL53L1 is detected)
  VL53L1::RangingMode ranging_mode = VL53L1::RangingMode::HIGH_PRECISION;

  // VL53L0X specific settings (used if VL53L0X is detected)
  bool io_2v8 = true;
  uint16_t timeout_ms = 500;
  uint32_t timing_budget_us = 200000;
  float signal_rate_limit_mcps = 0.25;
};

/**
 * @brief Detected sensor type for each slot
 */
enum class DetectedSensorType {
  NONE,      // No sensor detected
  VL53L0X,   // VL53L0X sensor detected at address
  VL53L1     // VL53L1 sensor detected at address
};

/**
 * @brief Array manager for multiple ToF sensors with automatic detection
 *
 * This class handles:
 * 1. UART->I2C mode switching workaround for TOF400F modules (VL53L1)
 * 2. I2C bus scanning to detect which sensors are present
 * 3. Automatic driver selection based on detected sensor type
 * 4. Unified reading interface for mixed sensor arrays
 */
class ToFSensorArray {
public:
  explicit ToFSensorArray(uint8_t num_sensors);
  ~ToFSensorArray() = default;

  /**
   * @brief Add and auto-detect a ToF sensor
   *
   * This function:
   * 1. Attempts UART->I2C mode switch (workaround for TOF400F)
   * 2. Scans I2C bus at configured address
   * 3. Detects sensor type (VL53L0X or VL53L1)
   * 4. Initializes appropriate driver
   *
   * @param index Sensor slot index (0-based)
   * @param cfg Configuration for this sensor slot
   * @return true if sensor detected and initialized, false otherwise
   */
  bool add_sensor(uint8_t index, const ToFSensorConfig& cfg);

  /**
   * @brief Update readings from all configured sensors
   *
   * @param readings Output vector to store readings
   * @return true if any sensor provided new data
   */
  bool update_readings(std::vector<SensorCommon::Reading>& readings);

  /**
   * @brief Get the detected sensor type for a given slot
   *
   * @param index Sensor slot index
   * @return Detected sensor type
   */
  DetectedSensorType get_detected_type(uint8_t index) const {
    if (index >= num_sensors_) return DetectedSensorType::NONE;
    return detected_types_[index];
  }

private:
  /**
   * @brief Apply UART->I2C mode switch workaround
   *
   * WORKAROUND: TOF400F modules (containing VL53L1) default to UART/Modbus mode.
   * This function sends a Modbus command via UART to enable I2C mode.
   *
   * NOTE: This command is sent even if no device is connected to UART,
   * which allows the code to work with both VL53L0X (no UART) and VL53L1 (UART).
   *
   * @param cfg Configuration containing UART pins
   * @return true if command sent successfully (does NOT guarantee device present)
   */
  bool apply_uart_to_i2c_workaround(const ToFSensorConfig& cfg);

  /**
   * @brief Scan I2C bus and detect sensor type at address
   *
   * @param cfg Configuration containing I2C settings
   * @return Detected sensor type
   */
  DetectedSensorType scan_and_detect_sensor(const ToFSensorConfig& cfg);

  /**
   * @brief Perform full I2C bus scan to find all devices
   *
   * Scans addresses 0x08 to 0x77 and returns all responding devices.
   * This is useful for diagnostics after UART mode switch.
   *
   * @param cfg Configuration containing I2C settings
   * @param found_addresses Output vector of found device addresses
   * @return true if scan completed successfully
   */
  bool scan_i2c_bus(const ToFSensorConfig& cfg, std::vector<uint8_t>& found_addresses);

  /**
   * @brief Probe I2C bus for device at specific address
   *
   * @param bus_handle I2C bus handle
   * @param address 7-bit I2C address
   * @param freq_hz I2C frequency
   * @return true if device responds
   */
  bool probe_i2c_address(i2c_master_bus_handle_t bus_handle,
                         uint8_t address,
                         uint32_t freq_hz = 100000);

  /**
   * @brief Get name of known I2C device by address
   *
   * @param address 7-bit I2C address
   * @return Device name string or "Unknown device"
   */
  static const char* get_device_name(uint8_t address);

  /**
   * @brief Calculate Modbus CRC16
   */
  static uint16_t calculate_modbus_crc(const uint8_t* data, size_t length);

  /**
   * @brief Build Modbus write command
   */
  static size_t build_modbus_write_command(uint8_t* buffer,
                                           uint8_t slave_addr,
                                           uint16_t reg_addr,
                                           uint16_t value);

  // Known I2C device database for identification
  struct DeviceInfo {
    uint8_t address;
    const char* name;
  };
  static const DeviceInfo KNOWN_DEVICES[];
  static const size_t NUM_KNOWN_DEVICES;

private:
  std::vector<std::unique_ptr<VL53L0X>> vl53l0x_sensors_;
  std::vector<std::unique_ptr<VL53L1>> vl53l1_sensors_;
  std::vector<DetectedSensorType> detected_types_;
  uint8_t num_sensors_;
};

/**
 * @brief Manager for ToF sensor array with background reading task
 */
class ToFSensorManager {
public:
  static ToFSensorManager& instance();

  /**
   * @brief Configure ToF sensor array
   *
   * @param sensor_configs Array of sensor configurations
   * @param num_sensors Number of sensors
   * @return true on success
   */
  bool configure(const ToFSensorConfig* sensor_configs, uint8_t num_sensors);

  /**
   * @brief Start background reading task
   *
   * @param read_interval_ms Reading interval in milliseconds
   * @param priority Task priority
   * @return true on success
   */
  bool start_reading_task(uint32_t read_interval_ms, UBaseType_t priority);

  /**
   * @brief Get latest readings from all sensors
   *
   * @param readings Output vector for readings
   * @return true if data retrieved successfully
   */
  bool get_latest_readings(std::vector<SensorCommon::Reading>& readings);

  /**
   * @brief Pause reading task
   */
  void pause();

  /**
   * @brief Resume reading task
   */
  void resume();

  /**
   * @brief Stop reading task
   */
  void stop();

  /**
   * @brief Get detected sensor types
   *
   * @return Vector of detected sensor types
   */
  std::vector<DetectedSensorType> get_detected_types() const;

private:
  ToFSensorManager();
  ~ToFSensorManager();

  static void reading_task(void* param);

  struct TaskParams {
    ToFSensorManager* manager;
    uint32_t interval_ms;
  };

private:
  ToFSensorArray* array_;
  std::vector<SensorCommon::Reading> latest_readings_;
  SemaphoreHandle_t data_mutex_;
  TaskHandle_t task_handle_;
  bool running_;
  bool paused_;
};
