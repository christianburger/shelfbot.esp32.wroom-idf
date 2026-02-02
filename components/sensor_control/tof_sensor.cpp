#include "tof_sensor.hpp"
#include <memory>
#include "vl53l1_modbus.hpp"

const char* TofSensor::TAG = "TofSensor";

TofSensor::TofSensor(const Config& config)
    : config_(config),
      initialized_(false),
      continuous_mode_(false),
      last_read_time_us_(0) {

    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; i++) {
        drivers_[i] = nullptr;
        sensor_enabled_[i] = config.sensors[i].enabled;
        current_modes_[i] = config.sensors[i].mode;
    }
}

TofSensor::~TofSensor() {
    stop_continuous();

    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; i++) {
        if (drivers_[i] != nullptr) {
            delete static_cast<VL53L1_Modbus*>(drivers_[i]);
            drivers_[i] = nullptr;
        }
    }
}

esp_err_t TofSensor::initialize() {
    if (initialized_) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing %d ToF sensors...", SensorCommon::NUM_TOF_SENSORS);

    esp_err_t overall_status = ESP_OK;

    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; i++) {
        if (!sensor_enabled_[i]) {
            ESP_LOGI(TAG, "Sensor %d disabled, skipping", i);
            continue;
        }

        esp_err_t status = initialize_driver(i);
        if (status != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize sensor %d", i);
            overall_status = ESP_FAIL;
            sensor_enabled_[i] = false;
        }
    }

    initialized_ = (overall_status == ESP_OK);
    if (initialized_) {
        ESP_LOGI(TAG, "ToF sensors initialized successfully");
    } else {
        ESP_LOGE(TAG, "ToF sensors initialization failed");
    }

    return overall_status;
}

esp_err_t TofSensor::initialize_driver(uint8_t sensor_index) {
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS) {
        return ESP_ERR_INVALID_ARG;
    }

    const auto& sensor_config = config_.sensors[sensor_index];

    // Create driver configuration - driver handles all low-level details
    VL53L1_Modbus::Config driver_config;

    // Set configuration directly - no wrapper needed
    driver_config.uart_port = static_cast<uart_port_t>(sensor_config.uart_port);
    driver_config.uart_tx_pin = static_cast<gpio_num_t>(sensor_config.uart_tx_pin);
    driver_config.uart_rx_pin = static_cast<gpio_num_t>(sensor_config.uart_rx_pin);
    driver_config.modbus_slave_address = sensor_config.device_address;
    driver_config.timeout_ms = sensor_config.timeout_ms;
    driver_config.enable_continuous = true;  // Always enable for polling

    // Set ranging mode
    driver_config.ranging_mode = (sensor_config.mode == Mode::LONG_DISTANCE) ?
                                VL53L1_Modbus::RangingMode::LONG_DISTANCE :
                                VL53L1_Modbus::RangingMode::HIGH_PRECISION;

    // Create driver instance directly
    drivers_[sensor_index] = new VL53L1_Modbus(driver_config);
    VL53L1_Modbus* driver = static_cast<VL53L1_Modbus*>(drivers_[sensor_index]);

    // Initialize the driver
    const char* err = driver->init();
    if (err != nullptr) {
        ESP_LOGE(TAG, "Driver initialization failed for sensor %d: %s", sensor_index, err);
        delete driver;
        drivers_[sensor_index] = nullptr;
        return ESP_FAIL;
    }

    // Start continuous mode
    if (!driver->startContinuous()) {
        ESP_LOGW(TAG, "Failed to start continuous mode for sensor %d, but continuing", sensor_index);
    }

    ESP_LOGI(TAG, "Sensor %d initialized successfully", sensor_index);
    return ESP_OK;
}

esp_err_t TofSensor::destroy_driver(uint8_t sensor_index) {
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS || drivers_[sensor_index] == nullptr) {
        return ESP_OK;
    }

    VL53L1_Modbus* driver = static_cast<VL53L1_Modbus*>(drivers_[sensor_index]);
    delete driver;
    drivers_[sensor_index] = nullptr;

    return ESP_OK;
}

esp_err_t TofSensor::read_all(SensorCommon::TofMeasurement results[SensorCommon::NUM_TOF_SENSORS]) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t overall_status = ESP_OK;
    int64_t timestamp = esp_timer_get_time();

    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; i++) {
        if (!sensor_enabled_[i] || drivers_[i] == nullptr) {
            // Mark as disabled
            results[i].valid = false;
            results[i].status = 0xFF;  // Special status for disabled
            results[i].timestamp_us = timestamp;
            results[i].distance_mm = 0;
            results[i].timeout_occurred = false;
            continue;
        }

        esp_err_t status = read_sensor(i, results[i]);
        if (status != ESP_OK) {
            overall_status = ESP_FAIL;
            results[i].valid = false;
            results[i].status = 0xFE;  // Special status for read error
        }

        last_measurements_[i] = results[i];
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

    VL53L1_Modbus* driver = static_cast<VL53L1_Modbus*>(drivers_[sensor_index]);

    // Read directly from driver
    VL53L1_Modbus::Measurement driver_result;
    if (!driver->readContinuous(driver_result)) {
        ESP_LOGW(TAG, "Failed to read measurement from sensor %d", sensor_index);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Convert driver result to common format
    result.distance_mm = driver_result.distance_mm;
    result.valid = driver_result.valid;
    result.status = driver_result.range_status;
    result.timestamp_us = driver_result.timestamp_us;
    result.timeout_occurred = driver->timeoutOccurred();

    return ESP_OK;
}

esp_err_t TofSensor::set_mode(uint8_t sensor_index, Mode mode) {
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS ||
        !sensor_enabled_[sensor_index] ||
        drivers_[sensor_index] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    VL53L1_Modbus* driver = static_cast<VL53L1_Modbus*>(drivers_[sensor_index]);

    // Convert mode and set directly on driver
    VL53L1_Modbus::RangingMode driver_mode = (mode == Mode::LONG_DISTANCE) ?
                                            VL53L1_Modbus::RangingMode::LONG_DISTANCE :
                                            VL53L1_Modbus::RangingMode::HIGH_PRECISION;

    const char* err = driver->setRangingMode(driver_mode);
    if (err != nullptr) {
        ESP_LOGE(TAG, "Failed to set mode for sensor %d: %s", sensor_index, err);
        return ESP_FAIL;
    }

    current_modes_[sensor_index] = mode;
    config_.sensors[sensor_index].mode = mode;

    ESP_LOGI(TAG, "Sensor %d mode changed to %s", sensor_index,
             mode == Mode::LONG_DISTANCE ? "LONG_DISTANCE" : "HIGH_PRECISION");

    return ESP_OK;
}

TofSensor::Mode TofSensor::get_mode(uint8_t sensor_index) const {
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS) {
        return Mode::LONG_DISTANCE;  // Default
    }

    return current_modes_[sensor_index];
}

esp_err_t TofSensor::set_timeout(uint8_t sensor_index, uint16_t timeout_ms) {
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS ||
        !sensor_enabled_[sensor_index] ||
        drivers_[sensor_index] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    VL53L1_Modbus* driver = static_cast<VL53L1_Modbus*>(drivers_[sensor_index]);
    driver->setTimeout(timeout_ms);

    config_.sensors[sensor_index].timeout_ms = timeout_ms;

    ESP_LOGD(TAG, "Sensor %d timeout set to %d ms", sensor_index, timeout_ms);
    return ESP_OK;
}

uint16_t TofSensor::get_timeout(uint8_t sensor_index) const {
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS) {
        return 0;
    }

    return config_.sensors[sensor_index].timeout_ms;
}

esp_err_t TofSensor::start_continuous() {
    if (continuous_mode_) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Starting continuous mode for ToF sensors");

    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; i++) {
        if (sensor_enabled_[i] && drivers_[i] != nullptr) {
            VL53L1_Modbus* driver = static_cast<VL53L1_Modbus*>(drivers_[i]);
            if (!driver->startContinuous()) {
                ESP_LOGW(TAG, "Failed to start continuous mode for sensor %d", i);
            }
        }
    }

    continuous_mode_ = true;
    return ESP_OK;
}

esp_err_t TofSensor::stop_continuous() {
    if (!continuous_mode_) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping continuous mode for ToF sensors");

    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; i++) {
        if (sensor_enabled_[i] && drivers_[i] != nullptr) {
            VL53L1_Modbus* driver = static_cast<VL53L1_Modbus*>(drivers_[i]);
            driver->stopContinuous();
        }
    }

    continuous_mode_ = false;
    return ESP_OK;
}

bool TofSensor::is_continuous() const {
    return continuous_mode_;
}

esp_err_t TofSensor::self_test(uint8_t sensor_index) {
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS ||
        !sensor_enabled_[sensor_index] ||
        drivers_[sensor_index] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    VL53L1_Modbus* driver = static_cast<VL53L1_Modbus*>(drivers_[sensor_index]);

    // Check if sensor is responsive
    if (!driver->probe()) {
        ESP_LOGE(TAG, "Sensor %d self-test failed: probe failed", sensor_index);
        return ESP_FAIL;
    }

    // Run driver's self-test
    const char* err = driver->selfTest();
    if (err != nullptr) {
        ESP_LOGE(TAG, "Sensor %d self-test failed: %s", sensor_index, err);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sensor %d self-test passed", sensor_index);
    return ESP_OK;
}

bool TofSensor::probe(uint8_t sensor_index) {
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS ||
        !sensor_enabled_[sensor_index] ||
        drivers_[sensor_index] == nullptr) {
        return false;
    }

    VL53L1_Modbus* driver = static_cast<VL53L1_Modbus*>(drivers_[sensor_index]);
    return driver->probe();
}

bool TofSensor::timeout_occurred(uint8_t sensor_index) {
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS ||
        !sensor_enabled_[sensor_index] ||
        drivers_[sensor_index] == nullptr) {
        return false;
    }

    VL53L1_Modbus* driver = static_cast<VL53L1_Modbus*>(drivers_[sensor_index]);
    return driver->timeoutOccurred();
}

bool TofSensor::enable_sensor(uint8_t sensor_index, bool enable) {
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS) {
        return false;
    }

    if (enable == sensor_enabled_[sensor_index]) {
        return true;  // No change
    }

    if (enable) {
        // Enable the sensor
        esp_err_t err = initialize_driver(sensor_index);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable sensor %d", sensor_index);
            return false;
        }
        sensor_enabled_[sensor_index] = true;
        ESP_LOGI(TAG, "Sensor %d enabled", sensor_index);
    } else {
        // Disable the sensor
        destroy_driver(sensor_index);
        sensor_enabled_[sensor_index] = false;
        ESP_LOGI(TAG, "Sensor %d disabled", sensor_index);
    }

    return true;
}

bool TofSensor::is_sensor_enabled(uint8_t sensor_index) const {
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS) {
        return false;
    }

    return sensor_enabled_[sensor_index];
}

bool TofSensor::is_ready() const {
    if (!initialized_) {
        return false;
    }

    // Check if at least one sensor is ready
    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; i++) {
        if (sensor_enabled_[i] && drivers_[i] != nullptr) {
            VL53L1_Modbus* driver = static_cast<VL53L1_Modbus*>(drivers_[i]);
            if (driver->isReady()) {
                return true;
            }
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

    VL53L1_Modbus* driver = static_cast<VL53L1_Modbus*>(drivers_[sensor_index]);
    return driver->isReady();
}