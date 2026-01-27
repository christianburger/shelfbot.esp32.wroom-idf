#include "tof_sensor.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "../firmware_version/firmware_version.hpp"
#include <cinttypes>

static const char* TAG = "ToF_Sensor";

// ============================================================================
// ToFSensorArray Implementation
// ============================================================================

ToFSensorArray::ToFSensorArray(uint8_t num_sensors)
    : num_sensors_(num_sensors) {
    sensors_.resize(num_sensors);
}

bool ToFSensorArray::add_sensor(uint8_t index, const ToFSensorConfig& cfg) {
    if (index >= num_sensors_) {
        ESP_LOGE(TAG, "Sensor index %d out of bounds", index);
        return false;
    }

    ESP_LOGI(TAG, "Configuring ToF sensor %d at address 0x%02X", index, cfg.i2c_address);

    // Create VL53L0X configuration from ToFSensorConfig
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

    // Create sensor instance
    sensors_[index] = std::make_unique<VL53L0X>(vl_config);

    // Initialize sensor
    const char* error = sensors_[index]->init();
    if (error != nullptr) {
        ESP_LOGE(TAG, "Failed to initialize ToF sensor %d: %s", index, error);

        // Retry once with longer delay
        ESP_LOGI(TAG, "Retrying initialization after delay...");
        vTaskDelay(pdMS_TO_TICKS(200));

        error = sensors_[index]->init();
        if (error != nullptr) {
            ESP_LOGE(TAG, "Retry failed: %s", error);
            sensors_[index].reset();
            return false;
        }
    }

    ESP_LOGI(TAG, "ToF sensor %d initialized successfully", index);

    // Perform self-test
    error = sensors_[index]->selfTest();
    if (error != nullptr) {
        ESP_LOGW(TAG, "Self-test warning for sensor %d: %s", index, error);
    } else {
        ESP_LOGI(TAG, "Sensor %d self-test passed", index);
    }

    return true;
}

bool ToFSensorArray::update_readings(std::vector<SensorCommon::Reading>& readings) {
    readings.resize(num_sensors_);
    bool any_new_data = false;

    ESP_LOGD(TAG, "Updating readings for %d ToF sensors", num_sensors_);

    for (uint8_t i = 0; i < num_sensors_; ++i) {
        if (!sensors_[i] || !sensors_[i]->isReady()) {
            readings[i].distance_cm = SensorCommon::MAX_DISTANCE_CM;
            readings[i].valid = false;
            readings[i].status = 255;  // Not configured
            readings[i].timestamp_us = esp_timer_get_time();
            ESP_LOGW(TAG, "ToF Sensor %d not ready or not configured", i);
            continue;
        }

        // Read distance using the clean API
        ESP_LOGD(TAG, "Reading ToF sensor %d...", i);
        VL53L0X::MeasurementResult result;
        bool success = sensors_[i]->readSingle(result);

        readings[i].timestamp_us = result.timestamp_us;

        if (!success) {
            readings[i].distance_cm = SensorCommon::MAX_DISTANCE_CM;
            readings[i].valid = false;
            readings[i].status = 2;  // I2C error
            ESP_LOGW(TAG, "ToF Sensor %d read failed", i);
            continue;
        }

        if (result.timeout_occurred) {
            readings[i].distance_cm = SensorCommon::MAX_DISTANCE_CM;
            readings[i].valid = false;
            readings[i].status = 1;  // Timeout
            ESP_LOGW(TAG, "ToF Sensor %d timeout", i);
            continue;
        }

        // Convert mm to cm
        float distance_cm = static_cast<float>(result.distance_mm) / 10.0f;

        // Validate range
        readings[i].distance_cm = distance_cm;
        readings[i].valid = result.valid &&
                           (distance_cm >= SensorCommon::MIN_DISTANCE_CM &&
                            distance_cm <= SensorCommon::MAX_DISTANCE_CM);
        readings[i].status = readings[i].valid ? 0 : 3;  // 0 = OK, 3 = out of range

        if (readings[i].valid) {
            any_new_data = true;
            ESP_LOGI(TAG, "[ToF Sensor %d] Distance: %.1f cm (VALID)", i, distance_cm);
        } else {
            ESP_LOGW(TAG, "[ToF Sensor %d] Distance: %.1f cm (INVALID)", i, distance_cm);
        }
    }

    ESP_LOGD(TAG, "ToF sensor update complete, any new data: %s",
             any_new_data ? "YES" : "NO");

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

    FirmwareVersion fw_version;
    fw_version.print_firmware_version("ToFSensorManager");

    ESP_LOGI(TAG, "ToFSensorManager constructor complete");
}

ToFSensorManager::~ToFSensorManager() {
    stop();
    if (data_mutex_) {
        vSemaphoreDelete(data_mutex_);
    }
}

bool ToFSensorManager::configure(const ToFSensorConfig* sensor_configs, uint8_t num_sensors) {
    if (array_) {
        ESP_LOGW(TAG, "Manager already configured");
        return false;
    }

    array_ = std::make_unique<ToFSensorArray>(num_sensors);

    bool any_sensor_configured = false;
    for (uint8_t i = 0; i < num_sensors; i++) {
        if (array_->add_sensor(i, sensor_configs[i])) {
            any_sensor_configured = true;
            ESP_LOGI(TAG, "ToF sensor %d configured successfully", i);
        } else {
            ESP_LOGE(TAG, "Failed to add ToF sensor %d", i);
        }
    }

    latest_readings_.resize(num_sensors);

    if (any_sensor_configured) {
        ESP_LOGI(TAG, "ToF Manager configured with at least one sensor working");
        return true;
    } else {
        ESP_LOGW(TAG, "ToF Manager: No sensors could be configured");
        array_.reset();
        return false;
    }
}

bool ToFSensorManager::start_reading_task(uint32_t read_interval_ms, UBaseType_t priority) {
    if (!array_) {
        ESP_LOGE(TAG, "Cannot start task: array not configured");
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
