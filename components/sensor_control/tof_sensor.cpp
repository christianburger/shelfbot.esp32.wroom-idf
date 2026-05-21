// tof_sensor.cpp
#include "tof_sensor.hpp"

const char* TofSensor::TAG = "TofSensor";

TofSensor::TofSensor(const Config& config) : config_(config) {
    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; ++i) {
        sensor_enabled_[i] = config_.tof_configs[i].enabled;
        drivers_[i] = nullptr;
    }
}

TofSensor::~TofSensor() = default;

void TofSensor::setDriverFactory(std::function<std::unique_ptr<ITofDriver>(uint8_t)> factory) {
    driver_factory_ = std::move(factory);
}

esp_err_t TofSensor::initialize() {
    if (initialized_) return ESP_OK;
    if (!driver_factory_) {
        ESP_LOGE(TAG, "Driver factory not set");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing ToF sensors...");
    int success = 0;
    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; ++i) {
        if (!sensor_enabled_[i]) continue;
        drivers_[i] = driver_factory_(i);
        if (!drivers_[i]) {
            ESP_LOGE(TAG, "Failed to create driver for sensor %d", i);
            sensor_enabled_[i] = false;
            continue;
        }
        if (const char* err; (err = drivers_[i]->configure())) {
            ESP_LOGE(TAG, "configure: %s", err);
        } else if ((err = drivers_[i]->init())) {
            ESP_LOGE(TAG, "init: %s", err);
        } else if ((err = drivers_[i]->setup())) {
            ESP_LOGE(TAG, "setup: %s", err);
        } else if ((err = drivers_[i]->calibrate())) {
            ESP_LOGE(TAG, "calibrate: %s", err);
        } else if ((err = drivers_[i]->check())) {
            ESP_LOGE(TAG, "check: %s", err);
        } else {
            ESP_LOGI(TAG, "Sensor %d initialized", i);
            ++success;
            continue;
        }
        drivers_[i].reset();
        sensor_enabled_[i] = false;
    }
    initialized_ = (success > 0);
    return initialized_ ? ESP_OK : ESP_FAIL;
}

bool TofSensor::isReady() const {
    if (!initialized_) return false;
    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; ++i) {
        if (sensor_enabled_[i] && drivers_[i] && drivers_[i]->isReady()) return true;
    }
    return false;
}

esp_err_t TofSensor::readAll(SensorCommon::TofMeasurement results[SensorCommon::NUM_TOF_SENSORS]) {
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    int64_t timestamp = esp_timer_get_time();
    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; ++i) {
        if (!sensor_enabled_[i] || !drivers_[i]) {
            results[i].valid = false;
            results[i].status = 0xFF;
            results[i].timestamp_us = timestamp;
            results[i].distance_mm = 0;
            results[i].timeout_occurred = false;
            continue;
        }
        TofDriverMeasurement raw;
        if (drivers_[i]->readSensor(raw)) {
            results[i].distance_mm = raw.distance_mm;
            results[i].valid = raw.valid;
            results[i].status = raw.range_status;
            results[i].timestamp_us = raw.timestamp_us;
            results[i].timeout_occurred = raw.timeout_occurred;
            last_measurements_[i] = results[i];
        } else {
            results[i].valid = false;
            results[i].status = 0xFE;
        }
    }
    last_read_time_us_ = timestamp;
    return ESP_OK;
}

esp_err_t TofSensor::readSingle(uint8_t index, SensorCommon::TofMeasurement& result) {
    if (index >= SensorCommon::NUM_TOF_SENSORS || !sensor_enabled_[index] || !drivers_[index])
        return ESP_ERR_INVALID_ARG;
    TofDriverMeasurement raw;
    if (!drivers_[index]->readSensor(raw)) return ESP_ERR_INVALID_RESPONSE;
    result.distance_mm = raw.distance_mm;
    result.valid = raw.valid;
    result.status = raw.range_status;
    result.timestamp_us = raw.timestamp_us;
    result.timeout_occurred = raw.timeout_occurred;
    return ESP_OK;
}

esp_err_t TofSensor::startContinuous() { continuous_mode_ = true; return ESP_OK; }
esp_err_t TofSensor::stopContinuous()  { continuous_mode_ = false; return ESP_OK; }
bool TofSensor::probe(uint8_t index) const {
    return (index < SensorCommon::NUM_TOF_SENSORS && sensor_enabled_[index] && drivers_[index] && drivers_[index]->check() == nullptr);
}
bool TofSensor::isSensorReady(uint8_t index) const {
    return (index < SensorCommon::NUM_TOF_SENSORS && sensor_enabled_[index] && drivers_[index] && drivers_[index]->isReady());
}