#include "include/sensor_manager.hpp"
#include "include/sensor_control.hpp"

static const char* TAG = "SensorManager";


void SensorManager::monitor_task(void* arg) {
    SensorManager* instance = static_cast<SensorManager*>(arg);
    if (instance) {
        instance->monitor_loop();
    }
    vTaskDelete(nullptr);
}

void SensorManager::monitor_loop() {
    ESP_LOGI(TAG, "Sensor monitor loop started (independent of network connectivity)");

    while (monitor_task_running_) {
        SensorCommon::SensorDataPacket snapshot;
        if (get_latest_data(snapshot)) {
            ESP_LOGI(TAG,
                     "Snapshot us=%lld | US[%.1f, %.1f, %.1f, %.1f] cm | TOF[%u, %u, %u] mm valid[%d, %d, %d]",
                     static_cast<long long>(snapshot.timestamp_us),
                     snapshot.ultrasonic_readings[0].distance_cm,
                     snapshot.ultrasonic_readings[1].distance_cm,
                     snapshot.ultrasonic_readings[2].distance_cm,
                     snapshot.ultrasonic_readings[3].distance_cm,
                     snapshot.tof_measurements[0].distance_mm,
                     snapshot.tof_measurements[1].distance_mm,
                     snapshot.tof_measurements[2].distance_mm,
                     snapshot.tof_measurements[0].valid,
                     snapshot.tof_measurements[1].valid,
                     snapshot.tof_measurements[2].valid);
        } else {
            ESP_LOGW(TAG, "Sensor snapshot unavailable");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "Sensor monitor loop stopped");
}

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

    ESP_LOGI(TAG, "Sensor setup stage: create SensorControl");

    // Create and initialize sensor control
    sensor_control_ = std::make_unique<SensorControl>(config_with_callbacks);

    ESP_LOGI(TAG, "Sensor setup stage: initialize SensorControl");
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

    ESP_LOGI(TAG, "Sensor setup stage: start continuous measurement loop");
    esp_err_t err = sensor_control_->start_continuous();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start continuous reading: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Sensor reading started");

        if (!monitor_task_running_) {
            monitor_task_running_ = true;
            BaseType_t task_result = xTaskCreate(
                monitor_task,
                "sensor_monitor_task",
                4096,
                this,
                tskIDLE_PRIORITY + 1,
                &monitor_task_handle_
            );

            if (task_result != pdPASS) {
                ESP_LOGE(TAG, "Failed to create sensor monitor task");
                monitor_task_running_ = false;
                monitor_task_handle_ = nullptr;
            }
        }
    }
}

void SensorManager::stop() {
    monitor_task_running_ = false;

    if (monitor_task_handle_) {
        vTaskDelay(pdMS_TO_TICKS(100));
        monitor_task_handle_ = nullptr;
    }

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
