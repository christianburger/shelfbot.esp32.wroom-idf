#include "include/tof400f_modbus_sensor.hpp"
#include "esp_log.h"
#include "esp_timer.h"

const char* Tof400fModbusSensor::TAG = "TOF400F_MODBUS_SENSOR";

Tof400fModbusSensor::Tof400fModbusSensor(const Config& config)
    : config_(config), initialized_(false) {}

const char* Tof400fModbusSensor::init() {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Initializing TOF400F via MODBUS (UART-only mode)");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "UART Port: %d", config_.uart_port);
    ESP_LOGI(TAG, "TX Pin: GPIO%d", config_.tx_pin);
    ESP_LOGI(TAG, "RX Pin: GPIO%d", config_.rx_pin);
    ESP_LOGI(TAG, "Baud Rate: %lu", config_.baud_rate);
    ESP_LOGI(TAG, "MODBUS Address: 0x%02X", config_.modbus_address);

    // Configure MODBUS interface
    Tof400fModbus::Config modbus_config;
    modbus_config.modbus_address = config_.modbus_address;
    modbus_config.uart_config.uart_port = config_.uart_port;
    modbus_config.uart_config.tx_pin = config_.tx_pin;
    modbus_config.uart_config.rx_pin = config_.rx_pin;
    modbus_config.uart_config.baud_rate = config_.baud_rate;
    modbus_config.uart_config.data_bits = 8;
    modbus_config.uart_config.parity = 0;  // None
    modbus_config.uart_config.stop_bits = 1;
    modbus_config.uart_config.timeout_ms = config_.timeout_ms;
    modbus_config.uart_config.max_retries = 3;

    modbus_ = std::make_unique<Tof400fModbus>(modbus_config);

    const char* error = modbus_->init();
    if (error) {
        ESP_LOGE(TAG, "MODBUS initialization failed: %s", error);
        return error;
    }

    ESP_LOGI(TAG, "Step 1: MODBUS interface initialized");

    // Read and log current configuration
    ESP_LOGI(TAG, "Step 2: Reading current sensor configuration...");

    Tof400fModbus::RangingMode current_mode;
    error = modbus_->getRangingMode(current_mode);
    if (error) {
        ESP_LOGW(TAG, "Failed to read ranging mode: %s", error);
    } else {
        const char* mode_str = (current_mode == Tof400fModbus::RangingMode::HIGH_PRECISION)
                              ? "HIGH_PRECISION" : "LONG_DISTANCE";
        ESP_LOGI(TAG, "  Current ranging mode: %s", mode_str);
    }

    // Set desired ranging mode
    if (config_.ranging_mode != current_mode) {
        ESP_LOGI(TAG, "Step 3: Setting ranging mode...");
        const char* mode_str = (config_.ranging_mode == Tof400fModbus::RangingMode::HIGH_PRECISION)
                              ? "HIGH_PRECISION" : "LONG_DISTANCE";
        ESP_LOGI(TAG, "  Requested mode: %s", mode_str);

        error = modbus_->setRangingMode(config_.ranging_mode, true);
        if (error) {
            ESP_LOGE(TAG, "Failed to set ranging mode: %s", error);
            return error;
        }
        ESP_LOGI(TAG, "  Ranging mode set successfully");
    } else {
        ESP_LOGI(TAG, "Step 3: Ranging mode already correct");
    }

    // Perform self-test
    ESP_LOGI(TAG, "Step 4: Performing self-test...");
    std::string test_result = modbus_->selfTest();
    ESP_LOGI(TAG, "%s", test_result.c_str());

    initialized_ = true;

    // Test measurement
    ESP_LOGI(TAG, "Step 5: Testing measurement read...");
    SensorCommon::Reading test_reading;
    if (readSingle(test_reading)) {
        if (test_reading.valid) {
            ESP_LOGI(TAG, "  Test measurement: %.1f cm (VALID)", test_reading.distance_cm);
        } else {
            ESP_LOGW(TAG, "  Test measurement: %.1f cm (INVALID)", test_reading.distance_cm);
        }
    } else {
        ESP_LOGW(TAG, "  Test measurement failed");
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "TOF400F MODBUS initialization complete - SUCCESS");
    ESP_LOGI(TAG, "========================================");

    return nullptr;
}

bool Tof400fModbusSensor::isReady() const {
    return initialized_ && modbus_ && modbus_->isReady();
}

bool Tof400fModbusSensor::readSingle(SensorCommon::Reading& result) {
  if (!isReady()) {
    ESP_LOGE(TAG, "Sensor not ready");
    return false;
  }

  result.timestamp_us = esp_timer_get_time();

  // Read measurement result via Tof400fModbus wrapper
  uint16_t distance_mm = 0;
  const char* error = modbus_->readMeasurement(distance_mm);

  if (error) {
    ESP_LOGW(TAG, "Failed to read measurement: %s", error);
    result.valid = false;
    result.status = 2;
    result.distance_cm = SensorCommon::MAX_DISTANCE_CM;
    return false;
  }

  ESP_LOGD(TAG, "Raw measurement: %u mm", distance_mm);

  // Convert to centimeters
  float distance_cm = distance_mm / 10.0f;

  // Validate range
  bool is_in_valid_range = (distance_cm >= SensorCommon::MIN_DISTANCE_CM &&
                            distance_cm <= SensorCommon::MAX_DISTANCE_CM);
  bool is_special_value = (distance_mm >= 8190);  // "No target" value

  result.distance_cm = distance_cm;
  result.valid = is_in_valid_range && !is_special_value;
  result.status = is_special_value ? 255 : (is_in_valid_range ? 0 : 1);

  if (result.valid) {
    ESP_LOGD(TAG, "Measurement: %.1f cm (VALID)", distance_cm);
  } else if (is_special_value) {
    ESP_LOGD(TAG, "Measurement: No target detected");
  } else {
    ESP_LOGD(TAG, "Measurement: %.1f cm (OUT OF RANGE)", distance_cm);
  }

  return true;
}

const char* Tof400fModbusSensor::setRangingMode(Tof400fModbus::RangingMode mode) {
    if (!isReady()) {
        return "Sensor not ready";
    }

    config_.ranging_mode = mode;
    return modbus_->setRangingMode(mode, true);
}

const char* Tof400fModbusSensor::selfTest() {
    if (!isReady()) {
        return "Sensor not initialized";
    }

    return modbus_->selfTest().c_str();
}
