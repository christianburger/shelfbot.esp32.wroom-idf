#include "tof_sensor.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cinttypes>

static const char* TAG = "ToF_Sensor";

ToFSensorArray::ToFSensorArray(uint8_t num_sensors)
    : num_sensors_(num_sensors) {
    vl53l0x_sensors_.resize(num_sensors);
    vl53l1_sensors_.resize(num_sensors);  // Add this
    is_vl53l1_.resize(num_sensors, false); // Add this
}

bool ToFSensorArray::add_sensor(uint8_t index, const ToFSensorConfig& cfg) {
    if (index >= num_sensors_) {
        ESP_LOGE(TAG, "Sensor index %d out of bounds", index);
        return false;
    }

    ESP_LOGI(TAG, "Configuring ToF sensor %d (%s) at address 0x%02X",
             index, cfg.use_vl53l1 ? "VL53L1" : "VL53L0X", cfg.i2c_address);

    if (cfg.use_vl53l1) {
        // === VL53L1 CONFIGURATION ===
        is_vl53l1_[index] = true;

        VL53L1::Config vl_config = vl53l1_default_config(
            cfg.i2c_port,
            cfg.sda_pin,
            cfg.scl_pin,
            cfg.xshut_pin,
            cfg.uart_port,
            cfg.uart_tx_pin,
            cfg.uart_rx_pin
        );

        vl_config.i2c_address = cfg.i2c_address;
        vl_config.timeout_ms = cfg.timeout_ms;
        vl_config.ranging_mode = cfg.ranging_mode;
        vl_config.enable_i2c_mode = true;  // Enable UART->I2C switch

        vl53l1_sensors_[index] = std::make_unique<VL53L1>(vl_config);

        const char* error = vl53l1_sensors_[index]->init();
        if (error != nullptr) {
            ESP_LOGE(TAG, "Failed to initialize VL53L1 sensor %d: %s", index, error);
            vl53l1_sensors_[index].reset();
            return false;
        }

        ESP_LOGI(TAG, "VL53L1 sensor %d initialized successfully", index);

    } else {
        // === VL53L0X CONFIGURATION (existing code) ===
        is_vl53l1_[index] = false;

        VL53L0X::Config vl_config = vl53l0x_default_config(
            cfg.i2c_port,
            cfg.sda_pin,
            cfg.scl_pin,
            cfg.xshut_pin
        );

        vl_config.i2c_address = cfg.i2c_address;
        vl_config.io_2v8 = cfg.io_2v8;
        vl_config.timeout_ms = cfg.timeout_ms;
        vl_config.timing_budget_us = cfg.timing_budget_us;
        vl_config.signal_rate_limit_mcps = cfg.signal_rate_limit_mcps;

        vl53l0x_sensors_[index] = std::make_unique<VL53L0X>(vl_config);

        const char* error = vl53l0x_sensors_[index]->init();
        if (error != nullptr) {
            ESP_LOGE(TAG, "Failed to initialize VL53L0X sensor %d: %s", index, error);
            vl53l0x_sensors_[index].reset();
            return false;
        }

        ESP_LOGI(TAG, "VL53L0X sensor %d initialized successfully", index);
    }

    return true;
}

bool ToFSensorArray::update_readings(std::vector<SensorCommon::Reading>& readings) {
    readings.resize(num_sensors_);
    bool any_new_data = false;

    ESP_LOGD(TAG, "Updating readings for %d ToF sensors", num_sensors_);

    for (uint8_t i = 0; i < num_sensors_; ++i) {
        if (is_vl53l1_[i]) {
            // === VL53L1 READING ===
            if (!vl53l1_sensors_[i] || !vl53l1_sensors_[i]->isReady()) {
                readings[i].distance_cm = SensorCommon::MAX_DISTANCE_CM;
                readings[i].valid = false;
                readings[i].status = 255;
                readings[i].timestamp_us = esp_timer_get_time();
                ESP_LOGW(TAG, "VL53L1 Sensor %d not ready", i);
                continue;
            }

            VL53L1::MeasurementResult result;
            bool success = vl53l1_sensors_[i]->readSingle(result);

            readings[i].timestamp_us = result.timestamp_us;

            if (!success) {
                readings[i].distance_cm = SensorCommon::MAX_DISTANCE_CM;
                readings[i].valid = false;
                readings[i].status = 2;
                ESP_LOGW(TAG, "VL53L1 Sensor %d read failed", i);
                continue;
            }

            if (result.timeout_occurred) {
                readings[i].distance_cm = SensorCommon::MAX_DISTANCE_CM;
                readings[i].valid = false;
                readings[i].status = 1;
                ESP_LOGW(TAG, "VL53L1 Sensor %d timeout", i);
                continue;
            }

            float distance_cm = static_cast<float>(result.distance_mm) / 10.0f;
            bool is_in_valid_range = (distance_cm >= SensorCommon::MIN_DISTANCE_CM &&
                                       distance_cm <= SensorCommon::MAX_DISTANCE_CM);
            bool is_special_value = (result.distance_mm >= 8190);

            readings[i].distance_cm = distance_cm;
            readings[i].valid = result.valid && is_in_valid_range && !is_special_value;
            readings[i].status = result.range_status;

            if (readings[i].valid) {
                any_new_data = true;
                ESP_LOGI(TAG, "[VL53L1 Sensor %d] Distance: %.1f cm (VALID)", i, distance_cm);
            } else {
                if (is_special_value) {
                    ESP_LOGD(TAG, "[VL53L1 Sensor %d] No target detected", i);
                } else {
                    ESP_LOGW(TAG, "[VL53L1 Sensor %d] Distance: %.1f cm (INVALID)", i, distance_cm);
                }
            }

        } else {
            // === VL53L0X READING (existing code) ===
            if (!vl53l0x_sensors_[i] || !vl53l0x_sensors_[i]->isReady()) {
                readings[i].distance_cm = SensorCommon::MAX_DISTANCE_CM;
                readings[i].valid = false;
                readings[i].status = 255;
                readings[i].timestamp_us = esp_timer_get_time();
                ESP_LOGW(TAG, "VL53L0X Sensor %d not ready", i);
                continue;
            }

            VL53L0X::MeasurementResult result;
            bool success = vl53l0x_sensors_[i]->readSingle(result);

            // ... existing VL53L0X code ...
        }
    }

    return any_new_data;
}
