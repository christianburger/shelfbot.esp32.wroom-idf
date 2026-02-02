#include "include/sensor_manager.hpp"
#include "include/sensor_control.hpp"

static const char* TAG = "SensorManager";

void SensorManager::initialize(const SensorControl::Config& config) {
    if (initialized_) {
        ESP_LOGW(TAG, "Already initialized");
        return;
    }

    // Create mutex for thread-safe data access
    data_mutex_ = xSemaphoreCreateMutex();
    if (!data_mutex_) {
        ESP_LOGE(TAG, "Failed to create data mutex");
        return;
    }

    // Create a copy of the config to set up callbacks
    SensorControl::Config config_with_callbacks = config;

    // Ultrasonic callback
    config_with_callbacks.ultrasonic_callback =
        [this](const std::vector<uint16_t>& distances) {
            if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
                for (size_t i = 0; i < distances.size() && i < SensorCommon::NUM_ULTRASONIC_SENSORS; i++) {
                    latest_data_.ultrasonic_readings[i].distance_cm = distances[i] / 10.0f;
                }
                latest_data_.timestamp_us = esp_timer_get_time();
                xSemaphoreGive(data_mutex_);
            }
        };

    // ToF callback
    config_with_callbacks.tof_callback =
        [this](const SensorCommon::TofMeasurement* measurements) {
            if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
                for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; i++) {
                    latest_data_.tof_measurements[i] = measurements[i];
                }
                latest_data_.timestamp_us = esp_timer_get_time();
                xSemaphoreGive(data_mutex_);
            }
        };

    // Create and initialize sensor control
    sensor_control_ = std::make_unique<SensorControl>(config_with_callbacks);

    esp_err_t err = sensor_control_->initialize();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor control: %s", esp_err_to_name(err));
        sensor_control_.reset();
        vSemaphoreDelete(data_mutex_);
        data_mutex_ = nullptr;
        return;
    }

    // Initialize the latest_data_ structure
    for (int i = 0; i < SensorCommon::NUM_ULTRASONIC_SENSORS; i++) {
        latest_data_.ultrasonic_readings[i] = SensorCommon::Reading();
    }
    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; i++) {
        latest_data_.tof_measurements[i] = SensorCommon::TofMeasurement();
    }
    latest_data_.timestamp_us = 0;

    initialized_ = true;
    ESP_LOGI(TAG, "SensorManager initialized successfully");
}

void SensorManager::start() {
    if (!initialized_ || !sensor_control_) {
        ESP_LOGE(TAG, "Cannot start - not initialized");
        return;
    }

    esp_err_t err = sensor_control_->start_continuous();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start continuous reading: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Sensor reading started");
    }
}

void SensorManager::stop() {
    if (sensor_control_) {
        sensor_control_->stop_continuous();
    }
}

bool SensorManager::get_latest_data(SensorCommon::SensorDataPacket& data) {
    if (!initialized_ || !data_mutex_) {
        return false;
    }

    if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        data = latest_data_;
        xSemaphoreGive(data_mutex_);
        return true;
    }

    return false;
}
