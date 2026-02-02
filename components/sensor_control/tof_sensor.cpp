#include "tof_sensor.hpp"
#include <memory>
#include "vl53l1.hpp"
#include "i2c_scanner.hpp"

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
            delete static_cast<VL53L1*>(drivers_[i]);
            drivers_[i] = nullptr;
        }
    }
}

esp_err_t TofSensor::initialize() {
    if (initialized_) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "  ToF Sensor Initialization - Starting Diagnostics");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "");

    // ========================================
    // STEP 1: I2C BUS DIAGNOSTIC SCAN
    // ========================================
    ESP_LOGI(TAG, "STEP 1: Performing comprehensive I2C bus scan...");
    ESP_LOGI(TAG, "");

    if (!I2CScanner::scanVL53L1Bus()) {
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "╔════════════════════════════════════════════════════╗");
        ESP_LOGE(TAG, "║  FATAL: I2C bus scan failed                        ║");
        ESP_LOGE(TAG, "║                                                    ║");
        ESP_LOGE(TAG, "║  Possible causes:                                  ║");
        ESP_LOGE(TAG, "║  1. I2C pins misconfigured in driver               ║");
        ESP_LOGE(TAG, "║  2. Hardware I2C peripheral conflict               ║");
        ESP_LOGE(TAG, "║  3. GPIO pins already in use                       ║");
        ESP_LOGE(TAG, "║                                                    ║");
        ESP_LOGE(TAG, "║  Check: vl53l1.cpp hardware configuration defines  ║");
        ESP_LOGE(TAG, "╚════════════════════════════════════════════════════╝");
        ESP_LOGE(TAG, "");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "✓ STEP 1 Complete: I2C bus scan finished");
    ESP_LOGI(TAG, "");
    vTaskDelay(pdMS_TO_TICKS(100));

    // ========================================
    // STEP 2: DRIVER INITIALIZATION
    // ========================================
    ESP_LOGI(TAG, "STEP 2: Initializing VL53L1 sensor drivers...");
    ESP_LOGI(TAG, "Number of sensors to initialize: %d", SensorCommon::NUM_TOF_SENSORS);
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
            ESP_LOGE(TAG, "  Troubleshooting hints for sensor %d:", i);
            ESP_LOGE(TAG, "    - Verify VL53L1 was found at 0x29 in scan above");
            ESP_LOGE(TAG, "    - Check power supply to sensor (3.3V)");
            ESP_LOGE(TAG, "    - Verify I2C pull-up resistors present (4.7kΩ)");
            ESP_LOGE(TAG, "    - Check for I2C bus conflicts with other devices");
            ESP_LOGE(TAG, "    - Ensure sensor is not in reset/shutdown mode");
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
    ESP_LOGI(TAG, "STEP 2 Summary:");
    ESP_LOGI(TAG, "  Total sensors configured: %d", SensorCommon::NUM_TOF_SENSORS);
    ESP_LOGI(TAG, "  Successfully initialized: %d", successful_inits);
    ESP_LOGI(TAG, "  Failed initializations:   %d", SensorCommon::NUM_TOF_SENSORS - successful_inits);
    ESP_LOGI(TAG, "");

    if (successful_inits == 0) {
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "╔════════════════════════════════════════════════════╗");
        ESP_LOGE(TAG, "║  CRITICAL: No ToF sensors initialized successfully ║");
        ESP_LOGE(TAG, "║                                                    ║");
        ESP_LOGE(TAG, "║  System will continue but ToF sensors unavailable  ║");
        ESP_LOGE(TAG, "╚════════════════════════════════════════════════════╝");
        ESP_LOGE(TAG, "");
        return ESP_FAIL;
    }

    initialized_ = (overall_status == ESP_OK);

    if (initialized_) {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "═══════════════════════════════════════════════════════");
        ESP_LOGI(TAG, "  ✓ ToF Sensor Initialization COMPLETE");
        ESP_LOGI(TAG, "  All configured sensors ready for measurements");
        ESP_LOGI(TAG, "═══════════════════════════════════════════════════════");
        ESP_LOGI(TAG, "");
    } else {
        ESP_LOGW(TAG, "");
        ESP_LOGW(TAG, "═══════════════════════════════════════════════════════");
        ESP_LOGW(TAG, "  ⚠ ToF Sensor Initialization PARTIAL");
        ESP_LOGW(TAG, "  Some sensors failed - %d/%d operational", successful_inits, SensorCommon::NUM_TOF_SENSORS);
        ESP_LOGW(TAG, "═══════════════════════════════════════════════════════");
        ESP_LOGW(TAG, "");
    }

    return overall_status;
}

esp_err_t TofSensor::initialize_driver(uint8_t sensor_index) {
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS) {
        ESP_LOGE(TAG, "    ✗ Invalid sensor index: %d", sensor_index);
        return ESP_ERR_INVALID_ARG;
    }

    const auto& sensor_config = config_.sensors[sensor_index];

    ESP_LOGI(TAG, "    • Creating VL53L1 driver instance...");
    ESP_LOGI(TAG, "    • Mode: %s",
             sensor_config.mode == Mode::LONG_DISTANCE ? "LONG_DISTANCE" : "HIGH_PRECISION");
    ESP_LOGI(TAG, "    • Timeout: %d ms", sensor_config.timeout_ms);

    // Create VL53L1 configuration with defaults from driver
    VL53L1::Config driver_config = vl53l1_default_config();

    // Override mode and timeout from sensor config
    driver_config.ranging_mode = (sensor_config.mode == Mode::LONG_DISTANCE) ?
                                VL53L1::RangingMode::LONG_DISTANCE :
                                VL53L1::RangingMode::HIGH_PRECISION;
    driver_config.timeout_ms = sensor_config.timeout_ms;

    ESP_LOGI(TAG, "    • Driver configuration:");
    ESP_LOGI(TAG, "      - I2C Port: %d", driver_config.i2c_port);
    ESP_LOGI(TAG, "      - SDA Pin: GPIO%d", driver_config.sda_pin);
    ESP_LOGI(TAG, "      - SCL Pin: GPIO%d", driver_config.scl_pin);
    ESP_LOGI(TAG, "      - I2C Address: 0x%02X", driver_config.i2c_address);
    ESP_LOGI(TAG, "      - I2C Frequency: %lu Hz", (unsigned long)driver_config.i2c_freq_hz);

    // Create driver instance
    ESP_LOGI(TAG, "    • Instantiating VL53L1 driver...");
    drivers_[sensor_index] = new VL53L1(driver_config);
    VL53L1* driver = static_cast<VL53L1*>(drivers_[sensor_index]);

    // Initialize the driver
    ESP_LOGI(TAG, "    • Calling driver init()...");
    const char* err = driver->init();
    if (err != nullptr) {
        ESP_LOGE(TAG, "    ✗ Driver init() failed: %s", err);
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "    Diagnostic information:");
        ESP_LOGE(TAG, "      This typically means:");
        ESP_LOGE(TAG, "      • Sensor not responding on I2C bus");
        ESP_LOGE(TAG, "      • Wrong I2C address (expected 0x29)");
        ESP_LOGE(TAG, "      • I2C communication failure");
        ESP_LOGE(TAG, "      • Sensor in wrong mode or reset state");

        delete driver;
        drivers_[sensor_index] = nullptr;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "    ✓ Driver init() successful");

    // Start continuous mode if sensor is enabled
    if (sensor_config.enabled) {
        ESP_LOGI(TAG, "    • Starting continuous measurement mode...");
        if (!driver->startContinuous()) {
            ESP_LOGW(TAG, "    ⚠ Failed to start continuous mode (non-fatal)");
            ESP_LOGW(TAG, "      Will attempt to use single-shot mode instead");
        } else {
            ESP_LOGI(TAG, "    ✓ Continuous mode started");
        }
    }

    ESP_LOGI(TAG, "    ✓ Sensor %d fully operational", sensor_index);
    return ESP_OK;
}

esp_err_t TofSensor::destroy_driver(uint8_t sensor_index) {
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS || drivers_[sensor_index] == nullptr) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Destroying driver for sensor %d", sensor_index);
    VL53L1* driver = static_cast<VL53L1*>(drivers_[sensor_index]);
    delete driver;
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
            ESP_LOGD(TAG, "Sensor %d read failed", i);
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

    VL53L1* driver = static_cast<VL53L1*>(drivers_[sensor_index]);

    // Read from VL53L1 driver
    VL53L1::MeasurementResult driver_result;
    bool success = false;

    if (continuous_mode_) {
        success = driver->readContinuous(driver_result);
    } else {
        success = driver->readSingle(driver_result);
    }

    if (!success) {
        ESP_LOGD(TAG, "Failed to read measurement from sensor %d", sensor_index);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Convert driver result to common format
    result.distance_mm = driver_result.distance_mm;
    result.valid = driver_result.valid;
    result.status = driver_result.range_status;
    result.timestamp_us = driver_result.timestamp_us;
    result.timeout_occurred = driver_result.timeout_occurred;

    return ESP_OK;
}

esp_err_t TofSensor::set_mode(uint8_t sensor_index, Mode mode) {
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS ||
        !sensor_enabled_[sensor_index] ||
        drivers_[sensor_index] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    VL53L1* driver = static_cast<VL53L1*>(drivers_[sensor_index]);

    // Convert mode and set on driver
    VL53L1::RangingMode driver_mode = (mode == Mode::LONG_DISTANCE) ?
                                      VL53L1::RangingMode::LONG_DISTANCE :
                                      VL53L1::RangingMode::HIGH_PRECISION;

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

    VL53L1* driver = static_cast<VL53L1*>(drivers_[sensor_index]);
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

    ESP_LOGI(TAG, "Starting continuous mode for all ToF sensors...");

    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; i++) {
        if (sensor_enabled_[i] && drivers_[i] != nullptr) {
            VL53L1* driver = static_cast<VL53L1*>(drivers_[i]);
            ESP_LOGI(TAG, "  • Sensor %d: starting continuous...", i);
            if (!driver->startContinuous()) {
                ESP_LOGW(TAG, "  ⚠ Failed to start continuous mode for sensor %d", i);
            } else {
                ESP_LOGI(TAG, "  ✓ Sensor %d: continuous mode active", i);
            }
        }
    }

    continuous_mode_ = true;
    ESP_LOGI(TAG, "✓ Continuous mode enabled for all sensors");
    return ESP_OK;
}

esp_err_t TofSensor::stop_continuous() {
    if (!continuous_mode_) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping continuous mode for all ToF sensors...");

    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; i++) {
        if (sensor_enabled_[i] && drivers_[i] != nullptr) {
            VL53L1* driver = static_cast<VL53L1*>(drivers_[i]);
            driver->stopContinuous();
            ESP_LOGI(TAG, "  ✓ Sensor %d: continuous mode stopped", i);
        }
    }

    continuous_mode_ = false;
    ESP_LOGI(TAG, "✓ Continuous mode disabled");
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

    ESP_LOGI(TAG, "Running self-test for sensor %d...", sensor_index);
    VL53L1* driver = static_cast<VL53L1*>(drivers_[sensor_index]);

    // Check if sensor is responsive
    ESP_LOGI(TAG, "  • Probing sensor...");
    if (!driver->probe()) {
        ESP_LOGE(TAG, "  ✗ Sensor %d self-test failed: probe failed", sensor_index);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "  ✓ Probe successful");

    // Run driver's self-test
    ESP_LOGI(TAG, "  • Running driver self-test...");
    const char* err = driver->selfTest();
    if (err != nullptr) {
        ESP_LOGE(TAG, "  ✗ Sensor %d self-test failed: %s", sensor_index, err);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "✓ Sensor %d self-test PASSED", sensor_index);
    return ESP_OK;
}

bool TofSensor::probe(uint8_t sensor_index) {
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS ||
        !sensor_enabled_[sensor_index] ||
        drivers_[sensor_index] == nullptr) {
        return false;
    }

    VL53L1* driver = static_cast<VL53L1*>(drivers_[sensor_index]);
    return driver->probe();
}

bool TofSensor::timeout_occurred(uint8_t sensor_index) {
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS ||
        !sensor_enabled_[sensor_index] ||
        drivers_[sensor_index] == nullptr) {
        return false;
    }

    VL53L1* driver = static_cast<VL53L1*>(drivers_[sensor_index]);
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
        ESP_LOGI(TAG, "Enabling sensor %d...", sensor_index);
        // Enable the sensor
        esp_err_t err = initialize_driver(sensor_index);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "✗ Failed to enable sensor %d", sensor_index);
            return false;
        }
        sensor_enabled_[sensor_index] = true;
        ESP_LOGI(TAG, "✓ Sensor %d enabled", sensor_index);
    } else {
        ESP_LOGI(TAG, "Disabling sensor %d...", sensor_index);
        // Disable the sensor
        destroy_driver(sensor_index);
        sensor_enabled_[sensor_index] = false;
        ESP_LOGI(TAG, "✓ Sensor %d disabled", sensor_index);
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
            VL53L1* driver = static_cast<VL53L1*>(drivers_[i]);
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

    VL53L1* driver = static_cast<VL53L1*>(drivers_[sensor_index]);
    return driver->isReady();
}