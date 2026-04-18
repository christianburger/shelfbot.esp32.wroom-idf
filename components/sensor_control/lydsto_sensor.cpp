#include "lydsto_sensor.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include <cinttypes>

static const char* TAG = "Lydsto_Sensor";

LydstoSensorArray::LydstoSensorArray(uint8_t num_sensors)
    : num_sensors_(num_sensors) {
    sensors_.resize(num_sensors);
}

bool LydstoSensorArray::add_sensor(uint8_t index, const LydstoSensorConfig& cfg) {
    if (index >= num_sensors_) {
        ESP_LOGE(TAG, "Sensor index %d out of bounds", index);
        return false;
    }

    sensors_[index] = std::make_unique<LydstoLidar>(cfg.lidar_config);
    const char* error = sensors_[index]->init();
    if (error != nullptr) {
        ESP_LOGE(TAG, "Failed to initialize Lydsto sensor %d: %s", index, error);
        sensors_[index].reset();
        return false;
    }

    error = sensors_[index]->selfTest();
    if (error != nullptr) {
        ESP_LOGW(TAG, "Self-test warning for Lydsto sensor %d: %s", index, error);
    }

    return true;
}

bool LydstoSensorArray::update_readings(std::vector<SensorCommon::Reading>& readings) {
    readings.resize(num_sensors_);
    bool any_new_data = false;

    for (uint8_t i = 0; i < num_sensors_; ++i) {
        SensorCommon::Reading& reading = readings[i];
        reading.distance_cm = SensorCommon::MAX_DISTANCE_CM;
        reading.valid = false;
        reading.status = 255;
        reading.timestamp_us = esp_timer_get_time();

        if (!sensors_[i] || !sensors_[i]->isReady()) {
            continue;
        }

        LydstoLidar::MeasurementResult result;
        if (!sensors_[i]->readSingle(result)) {
            reading.status = 1; // frame not available / timeout
            continue;
        }

        if (result.valid) {
            reading.distance_cm = result.distance_cm;
            reading.valid = (result.distance_cm >= SensorCommon::MIN_DISTANCE_CM &&
                             result.distance_cm <= SensorCommon::MAX_DISTANCE_CM);
            reading.status = reading.valid ? 0 : 2;
            reading.timestamp_us = result.timestamp_us;
            any_new_data = any_new_data || reading.valid;
        } else {
            reading.status = result.status;
        }
    }

    return any_new_data;
}

LydstoSensorManager& LydstoSensorManager::instance() {
    static LydstoSensorManager instance;
    return instance;
}

LydstoSensorManager::LydstoSensorManager()
    : data_mutex_(nullptr)
    , task_handle_(nullptr)
    , running_(false)
    , paused_(false) {
    data_mutex_ = xSemaphoreCreateMutex();
}

LydstoSensorManager::~LydstoSensorManager() {
    stop();
    if (data_mutex_) {
        vSemaphoreDelete(data_mutex_);
    }
}

bool LydstoSensorManager::configure(const LydstoSensorConfig* sensor_configs, uint8_t num_sensors) {
    if (array_) {
        ESP_LOGW(TAG, "Manager already configured");
        return false;
    }

    array_ = std::make_unique<LydstoSensorArray>(num_sensors);

    bool any_sensor_configured = false;
    for (uint8_t i = 0; i < num_sensors; ++i) {
        if (array_->add_sensor(i, sensor_configs[i])) {
            any_sensor_configured = true;
            ESP_LOGI(TAG, "Lydsto sensor %d configured", i);
        }
    }

    latest_readings_.resize(num_sensors);

    if (!any_sensor_configured) {
        array_.reset();
        return false;
    }

    return true;
}

bool LydstoSensorManager::start_reading_task(uint32_t read_interval_ms, UBaseType_t priority) {
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
    if (xTaskCreate(reading_task, "lydsto_reading", 4096, params, priority, &task_handle_) != pdPASS) {
        delete params;
        running_ = false;
        return false;
    }

    ESP_LOGI(TAG, "Lydsto reading task started with %" PRIu32 " ms interval", read_interval_ms);
    return true;
}

void LydstoSensorManager::reading_task(void* param) {
    auto* params = static_cast<TaskParams*>(param);
    LydstoSensorManager* self = params->manager;
    const uint32_t interval_ms = params->interval_ms;
    delete params;

    std::vector<SensorCommon::Reading> readings(self->latest_readings_.size());

    while (self->running_) {
        if (!self->paused_) {
            if (self->array_->update_readings(readings)) {
                if (xSemaphoreTake(self->data_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
                    self->latest_readings_ = readings;
                    xSemaphoreGive(self->data_mutex_);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }

    vTaskDelete(nullptr);
}

bool LydstoSensorManager::get_latest_readings(std::vector<SensorCommon::Reading>& readings) {
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

void LydstoSensorManager::pause() {
    paused_ = true;
}

void LydstoSensorManager::resume() {
    paused_ = false;
}

void LydstoSensorManager::stop() {
    running_ = false;
    if (task_handle_) {
        vTaskDelay(pdMS_TO_TICKS(200));
        task_handle_ = nullptr;
    }
}
