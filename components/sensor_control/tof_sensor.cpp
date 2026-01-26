#include "tof_sensor.hpp"
#include "i2c_scanner.hpp"  // Add I2C scanner header
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include <cstring>
#include <cinttypes>
#include "../firmware_version/firmware_version.hpp"

static const char* TAG = "ToF_Sensor";

// ============================================================================
// ToFSensorArray Implementation
// ============================================================================

ToFSensorArray::ToFSensorArray(uint8_t num_sensors)
    : num_sensors_(num_sensors), i2c_initialized_(false) {
  sensors_.resize(num_sensors);
}

bool ToFSensorArray::add_sensor(uint8_t index, const ToFSensorConfig& cfg) {
    if (index >= num_sensors_) {
        ESP_LOGE(TAG, "Sensor index %d out of bounds", index);
        return false;
    }

    // First, try a quick probe to see if the device is present
    ESP_LOGI(TAG, "Checking for ToF sensor at address 0x%02X...", cfg.i2c_address);

    if (I2CScanner::probe(cfg.i2c_port, cfg.i2c_address, cfg.sda_pin, cfg.scl_pin)) {
        ESP_LOGI(TAG, "Sensor found at address 0x%02X", cfg.i2c_address);
    } else {
        ESP_LOGW(TAG, "Sensor not found at address 0x%02X. Performing full I2C scan...", cfg.i2c_address);

        // Perform full I2C scan
        std::vector<uint8_t> found_addresses;
        if (I2CScanner::scan(cfg.i2c_port, found_addresses, cfg.sda_pin, cfg.scl_pin)) {
            ESP_LOGI(TAG, "I2C scan completed. Found %zu device(s)", found_addresses.size());
            I2CScanner::printResults(found_addresses);

            // Check if the expected address is present
            bool address_found = false;
            for (auto addr : found_addresses) {
                if (addr == cfg.i2c_address) {
                    address_found = true;
                    break;
                }
            }

            if (!address_found) {
                ESP_LOGE(TAG, "Expected sensor address 0x%02X not found in I2C scan!", cfg.i2c_address);
                return false;
            }
        } else {
            ESP_LOGE(TAG, "I2C scan failed. Check I2C connections and power.");
            return false;
        }
    }

    // Initialize I2C if not already done for this port
    if (!i2c_initialized_) {
        i2c_config_t i2c_config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = cfg.sda_pin,
            .scl_io_num = cfg.scl_pin,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master = { .clk_speed = 100000 },  // Start with 100kHz for compatibility
            .clk_flags = 0,
        };

        esp_err_t err = i2c_param_config(cfg.i2c_port, &i2c_config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure I2C: %s", esp_err_to_name(err));
            return false;
        }

        err = i2c_driver_install(cfg.i2c_port, I2C_MODE_MASTER, 0, 0, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(err));
            return false;
        }

        i2c_initialized_ = true;
        ESP_LOGI(TAG, "I2C initialized on port %d (SDA:%d, SCL:%d, 100kHz)",
                 cfg.i2c_port, cfg.sda_pin, cfg.scl_pin);
    }

    // Create the sensor with the actual driver interface
    sensors_[index] = std::make_shared<VL53L0X>(
        cfg.i2c_port,
        cfg.xshut_pin,
        cfg.i2c_address,
        cfg.io_2v8
    );

    // Handle XSHUT pin if configured
    if (cfg.xshut_pin != GPIO_NUM_NC) {
        ESP_LOGI(TAG, "Using XSHUT pin GPIO%d to reset sensor", cfg.xshut_pin);
        gpio_config_t xshut_config = {
            .pin_bit_mask = (1ULL << cfg.xshut_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };

        if (gpio_config(&xshut_config) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to configure XSHUT pin, continuing without hardware reset");
        } else {
            // Reset the sensor using XSHUT
            gpio_set_level(cfg.xshut_pin, 0);
            vTaskDelay(pdMS_TO_TICKS(10));
            gpio_set_level(cfg.xshut_pin, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
            ESP_LOGI(TAG, "Sensor reset via XSHUT pin");
        }
    }

    // Initialize the sensor
    ESP_LOGI(TAG, "Initializing ToF sensor...");

    // Set a longer timeout for initialization (500ms)
    sensors_[index]->setTimeout(500);

    const char* error = sensors_[index]->init();
    if (error != nullptr) {
        ESP_LOGE(TAG, "Failed to initialize ToF sensor %d: %s", index, error);

        // Try one more time with a longer delay and reset
        ESP_LOGI(TAG, "Retrying initialization with longer delay...");
        vTaskDelay(pdMS_TO_TICKS(200));

        if (cfg.xshut_pin != GPIO_NUM_NC) {
            // Reset again
            gpio_set_level(cfg.xshut_pin, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
            gpio_set_level(cfg.xshut_pin, 1);
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        error = sensors_[index]->init();

        if (error != nullptr) {
            ESP_LOGE(TAG, "Retry failed: %s", error);
            sensors_[index].reset();
            return false;
        }
    }

    // Set a longer timeout for measurements (200ms) to account for calibration issues
    sensors_[index]->setTimeout(200);

    ESP_LOGI(TAG, "ToF sensor %d initialized successfully at address 0x%02X",
             index, cfg.i2c_address);

    // Try a test reading
    ESP_LOGI(TAG, "Performing test reading...");
    uint16_t test_distance = sensors_[index]->readRangeSingleMillimeters();
    if (sensors_[index]->i2cFail()) {
        ESP_LOGW(TAG, "Test reading failed with I2C error");
    } else if (sensors_[index]->timeoutOccurred()) {
        ESP_LOGW(TAG, "Test reading timed out");
        // Try one more test with longer timeout
        ESP_LOGI(TAG, "Retrying test reading with 500ms timeout...");
        sensors_[index]->setTimeout(500);
        test_distance = sensors_[index]->readRangeSingleMillimeters();

        if (sensors_[index]->timeoutOccurred()) {
            ESP_LOGW(TAG, "Test reading still times out even with 500ms");
        } else {
            ESP_LOGI(TAG, "Test reading successful with longer timeout: %d mm", test_distance);
        }
    } else {
        ESP_LOGI(TAG, "Test reading successful: %d mm", test_distance);
    }

    return true;
}

bool ToFSensorArray::update_readings(std::vector<SensorCommon::Reading>& readings) {
    readings.resize(num_sensors_);
    bool any_new_data = false;

    for (uint8_t i = 0; i < num_sensors_; ++i) {
        if (!sensors_[i]) {
            readings[i].distance_cm = SensorCommon::MAX_DISTANCE_CM;
            readings[i].valid = false;
            readings[i].status = 255;  // Not configured
            readings[i].timestamp_us = esp_timer_get_time();
            continue;
        }

        // Read distance using the actual driver interface
        uint16_t distance_mm = sensors_[i]->readRangeSingleMillimeters();

        readings[i].timestamp_us = esp_timer_get_time();

        // Check for I2C failure first
        if (sensors_[i]->i2cFail()) {
            readings[i].distance_cm = SensorCommon::MAX_DISTANCE_CM;
            readings[i].valid = false;
            readings[i].status = 2;  // I2C error
            ESP_LOGD(TAG, "Sensor %d I2C failure", i);
            continue;
        }

        // Check for timeout (65535 indicates timeout in the driver)
        if (distance_mm == 65535 || sensors_[i]->timeoutOccurred()) {
            readings[i].distance_cm = SensorCommon::MAX_DISTANCE_CM;
            readings[i].valid = false;
            readings[i].status = 1;  // Timeout
            ESP_LOGD(TAG, "Sensor %d timeout", i);
            continue;
        }

        // Convert mm to cm
        float distance_cm = static_cast<float>(distance_mm) / 10.0f;

        // Validate range
        readings[i].distance_cm = distance_cm;
        readings[i].valid = (distance_cm >= SensorCommon::MIN_DISTANCE_CM &&
                            distance_cm <= SensorCommon::MAX_DISTANCE_CM);
        readings[i].status = readings[i].valid ? 0 : 3;  // 0 = OK, 3 = out of range

        if (readings[i].valid) {
            any_new_data = true;
            ESP_LOGD(TAG, "Sensor %d: %.1f cm", i, distance_cm);
        }
    }

    return any_new_data;
}

// ============================================================================
// ToFSensorManager Implementation
// ============================================================================

ToFSensorManager& ToFSensorManager::instance() {
    static ToFSensorManager instance;
    return instance;
}

ToFSensorManager::ToFSensorManager()
    : array_(nullptr)
    , data_mutex_(nullptr)
    , task_handle_(nullptr)
    , running_(false)
    , paused_(false) {
  data_mutex_ = xSemaphoreCreateMutex();

  // OPTION 1: Using instance method
  FirmwareVersion fw_version;
  fw_version.print_firmware_version("ToFSensorManager");

  ESP_LOGI(TAG, "ToFSensorManager constructor complete");
}

ToFSensorManager::~ToFSensorManager() {
    stop();
    if (array_) delete array_;
    if (data_mutex_) vSemaphoreDelete(data_mutex_);
}

bool ToFSensorManager::configure(const ToFSensorConfig* sensor_configs, uint8_t num_sensors) {
    if (array_) {
        ESP_LOGW(TAG, "Manager already configured");
        return false;
    }

    array_ = new ToFSensorArray(num_sensors);

    bool any_sensor_configured = false;
    for (uint8_t i = 0; i < num_sensors; i++) {
        if (array_->add_sensor(i, sensor_configs[i])) {
            any_sensor_configured = true;
            ESP_LOGI(TAG, "ToF sensor %d configured successfully", i);
        } else {
            ESP_LOGE(TAG, "Failed to add ToF sensor %d", i);
            // Continue to try other sensors
        }
    }

    latest_readings_.resize(num_sensors);

    if (any_sensor_configured) {
        ESP_LOGI(TAG, "ToF Manager configured with at least one sensor working");
        return true;
    } else {
        ESP_LOGW(TAG, "ToF Manager: No sensors could be configured");
        delete array_;
        array_ = nullptr;
        return false;
    }
}

bool ToFSensorManager::start_reading_task(uint32_t read_interval_ms, UBaseType_t priority) {
    if (!array_) {
        ESP_LOGE(TAG, "Cannot start task: array is null");
        return false;
    }

    if (task_handle_) {
        ESP_LOGW(TAG, "Task already running");
        return false;
    }

    running_ = true;
    paused_ = false;

    TaskParams* params = new TaskParams{this, read_interval_ms};

    if (xTaskCreate(reading_task, "tof_reading", 4096, params, priority, &task_handle_) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ToF reading task");
        delete params;
        running_ = false;
        return false;
    }

    ESP_LOGI(TAG, "ToF reading task started with %" PRIu32 " ms interval", read_interval_ms);
    return true;
}

void ToFSensorManager::reading_task(void* param) {
    auto* params = static_cast<TaskParams*>(param);
    ToFSensorManager* self = params->manager;
    uint32_t interval_ms = params->interval_ms;
    delete params;

    std::vector<SensorCommon::Reading> readings(self->latest_readings_.size());

    while (self->running_) {
        if (!self->paused_) {
            // Update readings from all sensors
            if (self->array_->update_readings(readings)) {
                // Update shared data with mutex protection
                if (xSemaphoreTake(self->data_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
                    self->latest_readings_ = readings;
                    xSemaphoreGive(self->data_mutex_);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }

    ESP_LOGI(TAG, "ToF reading task exiting");
    vTaskDelete(NULL);
}

bool ToFSensorManager::get_latest_readings(std::vector<SensorCommon::Reading>& readings) {
    if (!array_) {
        return false;
    }

    if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        readings = latest_readings_;
        xSemaphoreGive(data_mutex_);
        return true;
    }

    return false;
}

void ToFSensorManager::pause() {
    paused_ = true;
    ESP_LOGI(TAG, "ToF reading paused");
}

void ToFSensorManager::resume() {
    paused_ = false;
    ESP_LOGI(TAG, "ToF reading resumed");
}

void ToFSensorManager::stop() {
    running_ = false;
    if (task_handle_) {
        vTaskDelay(pdMS_TO_TICKS(200));
        task_handle_ = nullptr;
    }
    ESP_LOGI(TAG, "ToF reading stopped");
}
