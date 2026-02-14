#include <idf_c_includes.hpp>
#include "sensor_control.hpp"
#include "ultrasonic_sensor.hpp"
#include "tof_sensor.hpp"
#include "sensor_common.hpp"
#include "firmware_version.hpp"

const char* SensorControl::TAG = "SensorControl";

// Constructor
SensorControl::SensorControl(const Config& config)
    : config_(config),
      ultrasonic_array_(nullptr),
      tof_sensor_(nullptr),
      initialized_(false),
      continuous_mode_(false),
      continuous_task_handle_(nullptr),
      data_mutex_(nullptr) {

    // Create mutex for thread-safe data access
    data_mutex_ = xSemaphoreCreateMutex();
    if (!data_mutex_) {
        ESP_LOGE(TAG, "Failed to create data mutex");
    }
}

SensorControl::~SensorControl() {
    stop_continuous();

    if (continuous_task_handle_) {
        vTaskDelete(continuous_task_handle_);
        continuous_task_handle_ = nullptr;
    }

    if (data_mutex_) {
        vSemaphoreDelete(data_mutex_);
        data_mutex_ = nullptr;
    }
}

esp_err_t SensorControl::initialize_ultrasonic() {
    if (config_.ultrasonic_configs.empty()) {
        ESP_LOGW(TAG, "No ultrasonic sensors configured");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing %zu ultrasonic sensor(s)...",
             config_.ultrasonic_configs.size());

    ultrasonic_array_ = std::make_unique<UltrasonicSensorArray>();

    for (size_t i = 0; i < config_.ultrasonic_configs.size(); i++) {
        const auto& uconfig = config_.ultrasonic_configs[i];

        UltrasonicSensorConfig sensor_config = {
            .trig_pin = static_cast<gpio_num_t>(uconfig.trig_pin),
            .echo_pin = static_cast<gpio_num_t>(uconfig.echo_pin),
            .collision_threshold_cm = 20.0f,
            .timeout_us = uconfig.timeout_us,
            .state = SENSOR_IDLE,
            .start_time = 0,
            .pulse_duration = 0
        };

        if (!ultrasonic_array_->add_sensor(i, sensor_config)) {
            ESP_LOGE(TAG, "Failed to add ultrasonic sensor %zu", i);
            return ESP_FAIL;
        }

        ESP_LOGD(TAG, "Added ultrasonic sensor %zu: TRIG=GPIO%d, ECHO=GPIO%d",
                i, uconfig.trig_pin, uconfig.echo_pin);
    }

    if (!ultrasonic_array_->init()) {
        ESP_LOGE(TAG, "Failed to initialize ultrasonic sensors");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Ultrasonic sensors initialized successfully");
    return ESP_OK;
}

esp_err_t SensorControl::initialize_tof() {
    ESP_LOGI(TAG, "Initializing %d TOF sensors...", SensorCommon::NUM_TOF_SENSORS);

    // Convert our config to TofSensor::Config
    TofSensor::Config tof_config;

    // Create and initialize ToF sensor manager
    tof_sensor_ = std::make_unique<TofSensor>(tof_config);
    esp_err_t err = tof_sensor_->initialize();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TOF sensor initialization failed");
        tof_sensor_.reset();
        return err;
    }

    ESP_LOGI(TAG, "TOF sensors initialized successfully");
    return ESP_OK;
}

esp_err_t SensorControl::initialize() {
    if (initialized_) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Firmware Version: %s", FirmwareVersion::get_version_string());
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "Initializing Unified Sensor Control");
    ESP_LOGI(TAG, "=========================================");

    esp_err_t err = initialize_ultrasonic();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ultrasonic initialization failed");
        return err;
    }

    err = initialize_tof();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TOF initialization failed");
        ESP_LOGW(TAG, "Continuing without TOF sensors");
    }

    initialized_ = true;

    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "Sensor Control Initialization Complete");
    ESP_LOGI(TAG, "=========================================");

    return ESP_OK;
}

bool SensorControl::is_ready() const {
    bool ultrasonic_ready = config_.ultrasonic_configs.empty() ||
                           (ultrasonic_array_ != nullptr);

    bool tof_ready = (tof_sensor_ == nullptr) ||
                    tof_sensor_->is_ready();

    return initialized_ && ultrasonic_ready && tof_ready;
}

esp_err_t SensorControl::read_ultrasonic(std::vector<uint16_t>& distances) {
    distances.clear();

    if (!ultrasonic_array_ || config_.ultrasonic_configs.empty()) {
        return ESP_ERR_INVALID_STATE;
    }

    std::vector<SensorCommon::Reading> readings;
    if (!ultrasonic_array_->read_all_single(readings, SensorCommon::DEFAULT_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "Failed to read ultrasonic sensors");
        return ESP_ERR_INVALID_RESPONSE;
    }

    for (const auto& reading : readings) {
        uint16_t distance_mm = static_cast<uint16_t>(reading.distance_cm * 10);
        distances.push_back(distance_mm);
    }

    return ESP_OK;
}

esp_err_t SensorControl::read_tof(SensorCommon::TofMeasurement results[SensorCommon::NUM_TOF_SENSORS]) {
    if (!tof_sensor_) {
        return ESP_ERR_INVALID_STATE;
    }

    return tof_sensor_->read_all(results);
}

esp_err_t SensorControl::read_tof_single(uint8_t sensor_index, SensorCommon::TofMeasurement& result) {
    if (!tof_sensor_) {
        return ESP_ERR_INVALID_STATE;
    }

    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS) {
        return ESP_ERR_INVALID_ARG;
    }

    return tof_sensor_->read_sensor(sensor_index, result);
}

esp_err_t SensorControl::read_all(std::vector<uint16_t>& ultrasonic_distances,
                                 SensorCommon::TofMeasurement tof_results[SensorCommon::NUM_TOF_SENSORS]) {
    esp_err_t err = ESP_OK;

    esp_err_t ultrasonic_err = read_ultrasonic(ultrasonic_distances);
    if (ultrasonic_err != ESP_OK && ultrasonic_array_) {
        err = ultrasonic_err;
    }

    esp_err_t tof_err = read_tof(tof_results);
    if (tof_err != ESP_OK && tof_sensor_) {
        err = tof_err;
    }

    return err;
}

void SensorControl::continuous_read_loop() {
    ESP_LOGI(TAG, "Starting continuous sensor reading...");

    TickType_t last_ultrasonic_wake = xTaskGetTickCount();
    TickType_t last_tof_wake = xTaskGetTickCount();

    while (continuous_mode_) {
        TickType_t now = xTaskGetTickCount();

        // Read all sensors and update latest data
        if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
            int64_t timestamp = esp_timer_get_time();

            // Read ultrasonic sensors
            std::vector<SensorCommon::Reading> ultrasonic_readings;
            if (ultrasonic_array_) {
                ultrasonic_array_->read_all_single(ultrasonic_readings, SensorCommon::DEFAULT_TIMEOUT_MS);
                for (size_t i = 0; i < ultrasonic_readings.size() && i < SensorCommon::NUM_ULTRASONIC_SENSORS; i++) {
                    latest_data_.ultrasonic_readings[i] = ultrasonic_readings[i];
                }
            }

            // Read ToF sensors
            SensorCommon::TofMeasurement tof_results[SensorCommon::NUM_TOF_SENSORS];
            if (tof_sensor_ && tof_sensor_->read_all(tof_results) == ESP_OK) {
                for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; i++) {
                    latest_data_.tof_measurements[i] = tof_results[i];
                }
            }

            latest_data_.timestamp_us = timestamp;

            xSemaphoreGive(data_mutex_);

            // Call callbacks if set
            if ((now - last_ultrasonic_wake) * portTICK_PERIOD_MS >= config_.ultrasonic_read_interval_ms) {
                if (config_.ultrasonic_callback) {
                    std::vector<uint16_t> distances_mm;
                    for (const auto& reading : ultrasonic_readings) {
                        distances_mm.push_back(static_cast<uint16_t>(reading.distance_cm * 10));
                    }
                    config_.ultrasonic_callback(distances_mm);
                }
                last_ultrasonic_wake = now;
            }

            if ((now - last_tof_wake) * portTICK_PERIOD_MS >= config_.tof_read_interval_ms) {
                if (config_.tof_callback) {
                    config_.tof_callback(latest_data_.tof_measurements);
                }
                last_tof_wake = now;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "Continuous reading stopped");
}

void SensorControl::continuous_read_task(void* arg) {
    SensorControl* instance = static_cast<SensorControl*>(arg);
    if (instance) {
        instance->continuous_read_loop();
    }
    vTaskDelete(nullptr);
}

esp_err_t SensorControl::start_continuous() {
    if (continuous_mode_) {
        return ESP_OK;
    }

    if (!is_ready()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (tof_sensor_) {
        esp_err_t err = tof_sensor_->start_continuous();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start TOF continuous mode");
            return err;
        }
    }

    continuous_mode_ = true;

    BaseType_t result = xTaskCreate(
        continuous_read_task,
        "sensor_read_task",
        4096,
        this,
        tskIDLE_PRIORITY + 1,
        &continuous_task_handle_
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create continuous reading task");
        continuous_mode_ = false;
        if (tof_sensor_) {
            tof_sensor_->stop_continuous();
        }
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Continuous reading started");
    return ESP_OK;
}

esp_err_t SensorControl::stop_continuous() {
    if (!continuous_mode_) {
        return ESP_OK;
    }

    continuous_mode_ = false;

    // Wait for task to finish
    if (continuous_task_handle_) {
        vTaskDelay(pdMS_TO_TICKS(100));
        continuous_task_handle_ = nullptr;
    }

    if (tof_sensor_) {
        tof_sensor_->stop_continuous();
    }

    ESP_LOGI(TAG, "Continuous reading stopped");
    return ESP_OK;
}

bool SensorControl::is_continuous() const {
    return continuous_mode_;
}

size_t SensorControl::get_ultrasonic_count() const {
    return config_.ultrasonic_configs.size();
}

bool SensorControl::is_tof_ready(uint8_t sensor_index) const {
    return tof_sensor_ && tof_sensor_->is_sensor_ready(sensor_index);
}

bool SensorControl::is_ultrasonic_ready() const {
    return ultrasonic_array_ != nullptr;
}

esp_err_t SensorControl::self_test() {
    esp_err_t overall_result = ESP_OK;

    if (tof_sensor_) {
        for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; i++) {
            if (tof_sensor_->probe(i)) {
                ESP_LOGI(TAG, "TOF sensor %d probe passed", i);
            } else {
                ESP_LOGE(TAG, "TOF sensor %d probe failed", i);
                overall_result = ESP_FAIL;
            }
        }
    }

    return overall_result;
}

bool SensorControl::tof_probe(uint8_t sensor_index) {
    return tof_sensor_ && tof_sensor_->probe(sensor_index);
}

bool SensorControl::get_latest_data(SensorCommon::SensorDataPacket* data) {
    if (!data || !data_mutex_) {
        return false;
    }

    if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        *data = latest_data_;
        xSemaphoreGive(data_mutex_);
        return true;
    }

    return false;
}
