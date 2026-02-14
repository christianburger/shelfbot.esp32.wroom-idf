#include "tof_sensor.hpp"

// Driver already included via header - no additional includes needed

const char* TofSensor::TAG = "TofSensor";

TofSensor::TofSensor(const Config& config)
    : config_(config), initialized_(false), continuous_mode_(false),
      last_read_time_us_(0) {
    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; i++) {
        drivers_[i] = nullptr;
        sensor_enabled_[i] = config.tof_configs[i].enabled;
    }
}

TofSensor::~TofSensor() {
    stop_continuous();
    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; i++) {
        if (drivers_[i]) {
            delete drivers_[i];
            drivers_[i] = nullptr;
        }
    }
}

esp_err_t TofSensor::initialize() {
    if (initialized_) return ESP_OK;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "  ToF Sensor Initialization");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "");

    esp_err_t overall_status = ESP_OK;
    int successful_inits = 0;

    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; i++) {
        ESP_LOGI(TAG, "  ┌─────────────────────────────────────────┐");
        ESP_LOGI(TAG, "  │ Sensor %d Initialization                │", i);
        ESP_LOGI(TAG, "  └─────────────────────────────────────────┘");

        if (!sensor_enabled_[i]) {
            ESP_LOGI(TAG, "  ➜ Status: DISABLED (skipping)");
            ESP_LOGI(TAG, "");
            continue;
        }

        ESP_LOGI(TAG, "  ➜ Status: ENABLED");
        ESP_LOGI(TAG, "  ➜ Attempting initialization...");

        esp_err_t status = initialize_driver(i);

        if (status != ESP_OK) {
            ESP_LOGE(TAG, "  ✗ FAILED to initialize sensor %d", i);
            ESP_LOGE(TAG, "");
            overall_status = ESP_FAIL;
            sensor_enabled_[i] = false;
        } else {
            ESP_LOGI(TAG, "  ✓ SUCCESS: Sensor %d initialized", i);
            successful_inits++;
            ESP_LOGI(TAG, "");
        }
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Initialization Summary:");
    ESP_LOGI(TAG, "  Total sensors configured: %d", SensorCommon::NUM_TOF_SENSORS);
    ESP_LOGI(TAG, "  Successfully initialized: %d", successful_inits);
    ESP_LOGI(TAG, "  Failed initializations:   %d", SensorCommon::NUM_TOF_SENSORS - successful_inits);
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "");

    if (successful_inits == 0) {
        ESP_LOGE(TAG, "CRITICAL: No ToF sensors initialized successfully");
        return ESP_FAIL;
    }

    initialized_ = (overall_status == ESP_OK);
    return overall_status;
}

esp_err_t TofSensor::initialize_driver(uint8_t sensor_index) {
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS) {
        ESP_LOGE(TAG, "    ✗ Invalid sensor index: %d", sensor_index);
        return ESP_ERR_INVALID_ARG;
    }

    // Create driver instance - NO configuration passed!
    ESP_LOGI(TAG, "    • Creating driver instance...");
    drivers_[sensor_index] = new TofDriver();
    TofDriver* driver = drivers_[sensor_index];

    // Standardized initialization sequence - identical for all drivers
    const char* err;

    ESP_LOGI(TAG, "    • Running configure()...");
    err = driver->configure();
    if (err) {
        ESP_LOGE(TAG, "    configure() failed: %s", err);
        delete driver;
        drivers_[sensor_index] = nullptr;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "    • Running init()...");
    err = driver->init();
    if (err) {
        ESP_LOGE(TAG, "    init() failed: %s", err);
        delete driver;
        drivers_[sensor_index] = nullptr;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "    • Running setup()...");
    err = driver->setup();
    if (err) {
        ESP_LOGE(TAG, "    setup() failed: %s", err);
        delete driver;
        drivers_[sensor_index] = nullptr;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "    • Running calibrate()...");
    err = driver->calibrate();
    if (err) {
        ESP_LOGE(TAG, "    calibrate() failed: %s", err);
        delete driver;
        drivers_[sensor_index] = nullptr;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "    • Running check()...");
    err = driver->check();
    if (err) {
        ESP_LOGE(TAG, "    check() failed: %s", err);
        delete driver;
        drivers_[sensor_index] = nullptr;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "    ✓ Sensor %d fully operational", sensor_index);
    return ESP_OK;
}

esp_err_t TofSensor::destroy_driver(uint8_t sensor_index) {
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS || drivers_[sensor_index] == nullptr) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Destroying driver for sensor %d", sensor_index);
    delete drivers_[sensor_index];
    drivers_[sensor_index] = nullptr;

    return ESP_OK;
}

esp_err_t TofSensor::read_all(SensorCommon::TofMeasurement results[SensorCommon::NUM_TOF_SENSORS]) {
    if (!initialized_) {
        ESP_LOGW(TAG, "read_all() called but not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t overall_status = ESP_OK;
    int64_t timestamp = esp_timer_get_time();

    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; i++) {
        if (!sensor_enabled_[i] || drivers_[i] == nullptr) {
            results[i].valid = false;
            results[i].status = 0xFF;
            results[i].timestamp_us = timestamp;
            results[i].distance_mm = 0;
            results[i].timeout_occurred = false;
            continue;
        }

        TofDriver::MeasurementResult driver_result{};
        bool success = drivers_[i]->read_sensor(driver_result);

        if (success) {
            results[i].distance_mm = driver_result.distance_mm;
            results[i].valid = driver_result.valid;
            results[i].status = driver_result.range_status;
            results[i].timestamp_us = driver_result.timestamp_us;
            results[i].timeout_occurred = driver_result.timeout_occurred;
            last_measurements_[i] = results[i];
        } else {
            results[i].valid = false;
            results[i].status = 0xFE;
            overall_status = ESP_FAIL;
            ESP_LOGD(TAG, "Sensor %d read failed", i);
        }
    }

    last_read_time_us_ = timestamp;
    return overall_status;
}

esp_err_t TofSensor::read_sensor(uint8_t sensor_index, SensorCommon::TofMeasurement& result) {
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS ||
        !sensor_enabled_[sensor_index] ||
        drivers_[sensor_index] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    TofDriver::MeasurementResult driver_result{};
    if (!drivers_[sensor_index]->read_sensor(driver_result)) {
        ESP_LOGD(TAG, "Failed to read measurement from sensor %d", sensor_index);
        return ESP_ERR_INVALID_RESPONSE;
    }

    result.distance_mm = driver_result.distance_mm;
    result.valid = driver_result.valid;
    result.status = driver_result.range_status;
    result.timestamp_us = driver_result.timestamp_us;
    result.timeout_occurred = driver_result.timeout_occurred;

    return ESP_OK;
}

bool TofSensor::is_ready() const {
    if (!initialized_) return false;

    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; i++) {
        if (sensor_enabled_[i] && drivers_[i] != nullptr) {
            if (drivers_[i]->isReady()) return true;
        }
    }
    return false;
}

bool TofSensor::is_sensor_ready(uint8_t sensor_index) const {
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS ||
        !sensor_enabled_[sensor_index] ||
        drivers_[sensor_index] == nullptr) {
        return false;
    }

    return drivers_[sensor_index]->isReady();
}

esp_err_t TofSensor::self_test(uint8_t sensor_index) {
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS ||
        !sensor_enabled_[sensor_index] ||
        drivers_[sensor_index] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Running self-test for sensor %d...", sensor_index);

    const char* err = drivers_[sensor_index]->check();
    if (err != nullptr) {
        ESP_LOGE(TAG, "Self-test failed: %s", err);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Self-test passed for sensor %d", sensor_index);
    return ESP_OK;
}

bool TofSensor::probe(uint8_t sensor_index) {
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS ||
        !sensor_enabled_[sensor_index] ||
        drivers_[sensor_index] == nullptr) {
        return false;
    }

    return (drivers_[sensor_index]->check() == nullptr);
}

esp_err_t TofSensor::start_continuous() {
    if (continuous_mode_) return ESP_OK;
    continuous_mode_ = true;
    ESP_LOGI(TAG, "Continuous mode enabled");
    return ESP_OK;
}

esp_err_t TofSensor::stop_continuous() {
    if (!continuous_mode_) return ESP_OK;
    continuous_mode_ = false;
    ESP_LOGI(TAG, "Continuous mode disabled");
    return ESP_OK;
}

bool TofSensor::is_continuous() const {
    return continuous_mode_;
}
