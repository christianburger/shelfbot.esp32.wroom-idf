// tof_sensor.cpp - Refactored with comprehensive debugging
#include "tof_sensor.hpp"
#include "esp_log.h"

static const char *TAG = "ToF_Array";
static const char *TAG_MGR = "ToF_Manager";

// Global flag to track if I2C bus has been initialized (prevent double init)
static bool g_i2c_bus_initialized[I2C_NUM_MAX] = {false};

// ============================================================================
// ToFSensorArray Implementation
// ============================================================================

ToFSensorArray::ToFSensorArray(uint8_t num_sensors)
    : num_sensors_(num_sensors),
      initialized_(false),
      i2c_installed_(false),
      i2c_port_(I2C_NUM_0),
      array_mutex_(nullptr) {

    ESP_LOGI(TAG, "┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
    ESP_LOGI(TAG, "▶ ToFSensorArray Constructor Called");
    ESP_LOGI(TAG, "  ├─ Requested sensors: %d", num_sensors);

    sensors_.resize(num_sensors);
    ESP_LOGI(TAG, "  ├─ Sensor vector resized to %d slots", num_sensors);

    array_mutex_ = xSemaphoreCreateMutex();
    if (array_mutex_) {
        ESP_LOGI(TAG, "  ├─ Array mutex created successfully");
    } else {
        ESP_LOGE(TAG, "  ├─ ❌ FAILED to create array mutex!");
    }

    ESP_LOGI(TAG, "  └─ Constructor complete");
    ESP_LOGI(TAG, "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛\n");
}

ToFSensorArray::~ToFSensorArray() {
    ESP_LOGI(TAG, "▶ ToFSensorArray Destructor Called");

    if (array_mutex_) {
        ESP_LOGI(TAG, "  ├─ Deleting array mutex");
        vSemaphoreDelete(array_mutex_);
        ESP_LOGI(TAG, "  └─ Mutex deleted");
    }

    if (i2c_installed_) {
        ESP_LOGI(TAG, "  ├─ Uninstalling I2C driver on port %d", i2c_port_);
        esp_err_t err = i2c_driver_delete(i2c_port_);
        ESP_LOGI(TAG, "  └─ I2C driver delete result: %s", esp_err_to_name(err));
    }
}

bool ToFSensorArray::init(const ToFArrayConfig& array_config) {
    ESP_LOGI(TAG, "┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
    ESP_LOGI(TAG, "▶ ToFSensorArray::init() - I2C Bus Initialization");
    ESP_LOGI(TAG, "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛");

    if (initialized_) {
        ESP_LOGW(TAG, "  ⚠ Array already initialized, skipping");
        ESP_LOGI(TAG, "  ⚠ I2C Port %d already configured with:", i2c_port_);
        ESP_LOGI(TAG, "     SDA: GPIO %d, SCL: GPIO %d", array_config.sda_gpio, array_config.scl_gpio);
        return true;
    }

    // Check global flag - prevent multiple initializations across different instances
    if (g_i2c_bus_initialized[array_config.i2c_port]) {
        ESP_LOGW(TAG, "  ⚠ GLOBAL CHECK: I2C Port %d was already initialized by another instance!", array_config.i2c_port);
        ESP_LOGW(TAG, "  ⚠ Skipping GPIO configuration to prevent conflicts");
        ESP_LOGW(TAG, "  ⚠ Using existing I2C bus configuration");
        initialized_ = true;
        i2c_installed_ = false; // Don't try to delete driver we didn't create
        i2c_port_ = array_config.i2c_port;
        return true;
    }

    // TROUBLESHOOTING: Print GPIO configuration before applying
    ESP_LOGI(TAG, "\n  [TROUBLESHOOTING] GPIO Pin Configuration:");
    ESP_LOGI(TAG, "  ├─ SDA will be configured on GPIO %d", array_config.sda_gpio);
    ESP_LOGI(TAG, "  └─ SCL will be configured on GPIO %d", array_config.scl_gpio);

    ESP_LOGI(TAG, "  Configuration Parameters:");
    ESP_LOGI(TAG, "  ├─ I2C Port: %d", array_config.i2c_port);
    ESP_LOGI(TAG, "  ├─ SDA GPIO: %d", array_config.sda_gpio);
    ESP_LOGI(TAG, "  ├─ SCL GPIO: %d", array_config.scl_gpio);
    ESP_LOGI(TAG, "  ├─ Frequency: %lu Hz", (unsigned long)array_config.i2c_freq_hz);
    ESP_LOGI(TAG, "  ├─ Pullups: %s", array_config.enable_pullups ? "ENABLED" : "DISABLED");
    ESP_LOGI(TAG, "  └─ Num Sensors: %d", array_config.num_sensors);

    i2c_port_ = array_config.i2c_port;

    // Check if I2C driver is already installed on this port
    ESP_LOGI(TAG, "\n  [PRE-CHECK] Verifying I2C Port Status");
    esp_err_t test_err = i2c_driver_delete(array_config.i2c_port);
    if (test_err == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "  ├─ ✓ I2C Port %d is free (not installed)", array_config.i2c_port);
    } else if (test_err == ESP_OK) {
        ESP_LOGW(TAG, "  ├─ ⚠ I2C Port %d was already installed, removed it", array_config.i2c_port);
        ESP_LOGW(TAG, "  └─ This indicates a previous incomplete initialization");
        vTaskDelay(pdMS_TO_TICKS(100)); // Give time for cleanup
    } else {
        ESP_LOGE(TAG, "  ├─ ❌ Unexpected error checking I2C port: %s", esp_err_to_name(test_err));
    }

    // Step 1: Configure I2C parameters
    ESP_LOGI(TAG, "\n  [STEP 1] Configuring I2C Parameters");
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = array_config.sda_gpio;
    conf.scl_io_num = array_config.scl_gpio;
    conf.sda_pullup_en = array_config.enable_pullups ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    conf.scl_pullup_en = array_config.enable_pullups ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = array_config.i2c_freq_hz;
    conf.clk_flags = 0;

    ESP_LOGI(TAG, "  ├─ Calling i2c_param_config()...");
    esp_err_t err = i2c_param_config(array_config.i2c_port, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "  ├─ ❌ i2c_param_config FAILED!");
        ESP_LOGE(TAG, "  ├─ Error: %s (0x%X)", esp_err_to_name(err), err);
        ESP_LOGE(TAG, "  └─ Aborting initialization");
        return false;
    }
    ESP_LOGI(TAG, "  └─ ✓ i2c_param_config SUCCESS");

    // Step 2: Install I2C driver
    ESP_LOGI(TAG, "\n  [STEP 2] Installing I2C Driver");
    ESP_LOGI(TAG, "  ├─ Port: %d", array_config.i2c_port);
    ESP_LOGI(TAG, "  ├─ Mode: I2C_MODE_MASTER");
    ESP_LOGI(TAG, "  ├─ RX Buffer: 0 (not used in master mode)");
    ESP_LOGI(TAG, "  ├─ TX Buffer: 0 (not used in master mode)");
    ESP_LOGI(TAG, "  ├─ Flags: 0");
    ESP_LOGI(TAG, "  ├─ Calling i2c_driver_install()...");

    err = i2c_driver_install(array_config.i2c_port, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "  ├─ ❌ i2c_driver_install FAILED!");
        ESP_LOGE(TAG, "  ├─ Error: %s (0x%X)", esp_err_to_name(err), err);

        if (err == ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "  ├─ Driver already installed or invalid state");
        } else if (err == ESP_ERR_NO_MEM) {
            ESP_LOGE(TAG, "  ├─ Out of memory");
        }

        ESP_LOGE(TAG, "  └─ Aborting initialization");
        return false;
    }
    ESP_LOGI(TAG, "  └─ ✓ i2c_driver_install SUCCESS");

    i2c_installed_ = true;

    // Step 3: Set I2C timeout
    ESP_LOGI(TAG, "\n  [STEP 3] Setting I2C Timeout");
    ESP_LOGI(TAG, "  ├─ Timeout value: 80000 ticks (for clock stretching)");
    ESP_LOGI(TAG, "  ├─ Calling i2c_set_timeout()...");

    err = i2c_set_timeout(array_config.i2c_port, 80000);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "  ├─ ⚠ i2c_set_timeout WARNING: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "  └─ ✓ i2c_set_timeout SUCCESS");
    }

    // Step 4: Enable I2C filter
    ESP_LOGI(TAG, "\n  [STEP 4] Enabling I2C Filter");
    ESP_LOGI(TAG, "  ├─ Filter threshold: 5 APB clock cycles");
    ESP_LOGI(TAG, "  ├─ Calling i2c_filter_enable()...");

    err = i2c_filter_enable(array_config.i2c_port, 5);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "  ├─ ⚠ i2c_filter_enable WARNING: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "  └─ ✓ i2c_filter_enable SUCCESS");
    }

    // TROUBLESHOOTING: Verify GPIO configuration after I2C setup
    ESP_LOGI(TAG, "\n  [TROUBLESHOOTING] GPIO Configuration Complete:");
    ESP_LOGI(TAG, "  ├─ SDA configured on GPIO %d for I2C Port %d", array_config.sda_gpio, array_config.i2c_port);
    ESP_LOGI(TAG, "  └─ SCL configured on GPIO %d for I2C Port %d", array_config.scl_gpio, array_config.i2c_port);

    initialized_ = true;
    g_i2c_bus_initialized[array_config.i2c_port] = true; // Mark globally as initialized

    ESP_LOGI(TAG, "\n┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
    ESP_LOGI(TAG, "✓ I2C Bus Initialization COMPLETE");
    ESP_LOGI(TAG, "✓ Global flag set: I2C Port %d is now marked as configured", array_config.i2c_port);
    ESP_LOGI(TAG, "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛\n");

    return true;
}

bool ToFSensorArray::add_sensor(uint8_t index, const ToFSensorConfig& sensor_config) {
    ESP_LOGI(TAG, "┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
    ESP_LOGI(TAG, "▶ ToFSensorArray::add_sensor() - Sensor #%d", index);
    ESP_LOGI(TAG, "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛");

    if (index >= num_sensors_) {
        ESP_LOGE(TAG, "  ❌ Invalid sensor index: %d (max: %d)", index, num_sensors_ - 1);
        return false;
    }

    ESP_LOGI(TAG, "  Sensor Configuration:");
    ESP_LOGI(TAG, "  ├─ Index: %d", index);
    ESP_LOGI(TAG, "  ├─ I2C Address: 0x%02X", sensor_config.i2c_address);
    ESP_LOGI(TAG, "  ├─ XSHUT GPIO: %d", sensor_config.xshut_gpio);
    ESP_LOGI(TAG, "  ├─ INT GPIO: %d", sensor_config.int_gpio);
    ESP_LOGI(TAG, "  ├─ Range: %d mm", sensor_config.range_mm);
    ESP_LOGI(TAG, "  └─ Timing Budget: %lu ms", (unsigned long)sensor_config.timing_budget_ms);

    // Step 1: Handle XSHUT pin if configured
    if (sensor_config.xshut_gpio != GPIO_NUM_NC) {
        ESP_LOGI(TAG, "\n  [STEP 1] Configuring XSHUT Pin");
        ESP_LOGI(TAG, "  ├─ GPIO: %d", sensor_config.xshut_gpio);
        ESP_LOGI(TAG, "  ├─ Resetting pin...");
        gpio_reset_pin(sensor_config.xshut_gpio);

        ESP_LOGI(TAG, "  ├─ Setting LOW (sensor shutdown)...");
        gpio_set_level(sensor_config.xshut_gpio, 0);

        ESP_LOGI(TAG, "  ├─ Configuring as output...");
        gpio_set_direction(sensor_config.xshut_gpio, GPIO_MODE_OUTPUT);

        ESP_LOGI(TAG, "  ├─ Setting drive capability...");
        gpio_set_drive_capability(sensor_config.xshut_gpio, GPIO_DRIVE_CAP_3);

        ESP_LOGI(TAG, "  ├─ Waiting 100ms (sensor shutdown time)...");
        vTaskDelay(pdMS_TO_TICKS(100));

        ESP_LOGI(TAG, "  ├─ Setting HIGH (sensor power-up)...");
        gpio_set_level(sensor_config.xshut_gpio, 1);

        ESP_LOGI(TAG, "  ├─ Waiting 10ms (boot time)...");
        vTaskDelay(pdMS_TO_TICKS(10));

        ESP_LOGI(TAG, "  └─ ✓ XSHUT sequence complete");
    } else {
        ESP_LOGI(TAG, "\n  [STEP 1] XSHUT Pin: GPIO_NUM_NC (not used)");
    }

    // Step 2: Create VL53L0X instance
    ESP_LOGI(TAG, "\n  [STEP 2] Creating VL53L0X Instance");
    ESP_LOGI(TAG, "  ├─ Calling VL53L0X constructor...");
    ESP_LOGI(TAG, "  ├─ Parameters:");
    ESP_LOGI(TAG, "  │  ├─ I2C Port: %d", sensor_config.i2c_port);
    ESP_LOGI(TAG, "  │  ├─ XSHUT: %d", sensor_config.xshut_gpio);
    ESP_LOGI(TAG, "  │  ├─ Address: 0x%02X", sensor_config.i2c_address);
    ESP_LOGI(TAG, "  │  └─ IO Mode: 2V8");

    // FIXED: Constructor now takes 4 arguments: port, xshut, address, io_2v8
    auto sensor = std::make_shared<VL53L0X>(
        sensor_config.i2c_port,
        sensor_config.xshut_gpio,
        sensor_config.i2c_address,
        true  // io_2v8
    );

    if (!sensor) {
        ESP_LOGE(TAG, "  ├─ ❌ Failed to allocate VL53L0X instance!");
        ESP_LOGE(TAG, "  └─ Out of memory?");
        return false;
    }
    ESP_LOGI(TAG, "  └─ ✓ VL53L0X instance created");

    // Step 3: Initialize sensor
    ESP_LOGI(TAG, "\n  [STEP 3] Initializing VL53L0X Sensor");
    ESP_LOGI(TAG, "  ├─ This will perform full sensor initialization sequence");
    ESP_LOGI(TAG, "  ├─ Including: data init, static init, ref calibration");
    ESP_LOGI(TAG, "  ├─ Calling sensor->init()...");

    const char* err_str = sensor->init();
    if (err_str != nullptr) {
        ESP_LOGE(TAG, "  ├─ ❌ VL53L0X init FAILED!");
        ESP_LOGE(TAG, "  ├─ Error: %s", err_str);
        ESP_LOGE(TAG, "  ├─ Possible causes:");
        ESP_LOGE(TAG, "  │  ├─ I2C communication failure");
        ESP_LOGE(TAG, "  │  ├─ Wrong I2C address");
        ESP_LOGE(TAG, "  │  ├─ Sensor not powered");
        ESP_LOGE(TAG, "  │  ├─ SDA/SCL wiring issue");
        ESP_LOGE(TAG, "  │  └─ Sensor hardware fault");
        ESP_LOGE(TAG, "  └─ Aborting sensor addition");
        return false;
    }
    ESP_LOGI(TAG, "  └─ ✓ Sensor initialization SUCCESS");

    // Step 4: Configure timing budget
    ESP_LOGI(TAG, "\n  [STEP 4] Configuring Measurement Timing Budget");
    ESP_LOGI(TAG, "  ├─ Requested: %lu ms", (unsigned long)sensor_config.timing_budget_ms);
    ESP_LOGI(TAG, "  ├─ Converting to microseconds: %lu us", (unsigned long)(sensor_config.timing_budget_ms * 1000));
    ESP_LOGI(TAG, "  ├─ Calling setMeasurementTimingBudget()...");

    const char* budget_err = sensor->setMeasurementTimingBudget(sensor_config.timing_budget_ms * 1000);
    if (budget_err != nullptr) {
        ESP_LOGW(TAG, "  ├─ ⚠ setMeasurementTimingBudget WARNING: %s", budget_err);
        ESP_LOGW(TAG, "  ├─ Sensor will use default timing budget");
    } else {
        uint32_t actual_budget = sensor->getMeasurementTimingBudget();
        ESP_LOGI(TAG, "  ├─ ✓ Timing budget set successfully");
        ESP_LOGI(TAG, "  └─ Actual budget: %lu us (%.2f ms)", (unsigned long)actual_budget, actual_budget / 1000.0f);
    }

    // Step 5: Configure signal rate limit
    ESP_LOGI(TAG, "\n  [STEP 5] Configuring Signal Rate Limit");
    ESP_LOGI(TAG, "  ├─ Setting to: 0.25 MCPS");
    ESP_LOGI(TAG, "  ├─ Calling setSignalRateLimit()...");

    const char* rate_err = sensor->setSignalRateLimit(0.25);
    if (rate_err != nullptr) {
        ESP_LOGW(TAG, "  ├─ ⚠ setSignalRateLimit WARNING: %s", rate_err);
    } else {
        float actual_rate = sensor->getSignalRateLimit();
        ESP_LOGI(TAG, "  ├─ ✓ Signal rate limit set successfully");
        ESP_LOGI(TAG, "  └─ Actual rate: %.2f MCPS", actual_rate);
    }

    // Step 6: Store sensor in array
    ESP_LOGI(TAG, "\n  [STEP 6] Storing Sensor in Array");
    ESP_LOGI(TAG, "  ├─ Array slot: %d", index);
    sensors_[index] = sensor;
    ESP_LOGI(TAG, "  └─ ✓ Sensor stored");

    ESP_LOGI(TAG, "\n┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
    ESP_LOGI(TAG, "✓ Sensor #%d Addition COMPLETE", index);
    ESP_LOGI(TAG, "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛\n");

    return true;
}

bool ToFSensorArray::start_continuous() {
    ESP_LOGI(TAG, "┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
    ESP_LOGI(TAG, "▶ ToFSensorArray::start_continuous()");
    ESP_LOGI(TAG, "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛");

    ESP_LOGI(TAG, "  Starting continuous mode for %d sensor(s)", num_sensors_);

    for (uint8_t i = 0; i < num_sensors_; ++i) {
        ESP_LOGI(TAG, "\n  [Sensor #%d]", i);

        if (!sensors_[i]) {
            ESP_LOGW(TAG, "  ├─ ⚠ Sensor slot is NULL, skipping");
            continue;
        }

        ESP_LOGI(TAG, "  ├─ Calling startContinuous(0) [back-to-back mode]...");
        sensors_[i]->startContinuous(0);
        ESP_LOGI(TAG, "  └─ ✓ Continuous mode started");
    }

    ESP_LOGI(TAG, "\n┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
    ESP_LOGI(TAG, "✓ All sensors in continuous mode");
    ESP_LOGI(TAG, "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛\n");

    return true;
}

bool ToFSensorArray::read_all(std::vector<SensorCommon::Reading>& readings, uint32_t timeout_ms) {
    ESP_LOGD(TAG, "▶ read_all() - Acquiring mutex (timeout: %lu ms)", (unsigned long)timeout_ms);

    if (xSemaphoreTake(array_mutex_, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "  ❌ Mutex acquisition timeout!");
        return false;
    }

    ESP_LOGD(TAG, "  ✓ Mutex acquired, reading %d sensor(s)", num_sensors_);
    readings.resize(num_sensors_);

    for (uint8_t i = 0; i < num_sensors_; ++i) {
        ESP_LOGD(TAG, "\n  [Sensor #%d]", i);

        if (!sensors_[i]) {
            ESP_LOGD(TAG, "  ├─ Sensor NULL, marking invalid");
            readings[i].valid = false;
            readings[i].status = 255;
            readings[i].distance_cm = -1.0f;
            continue;
        }

        ESP_LOGD(TAG, "  ├─ Calling readRangeContinuousMillimeters()...");
        uint16_t mm = sensors_[i]->readRangeContinuousMillimeters();

        readings[i].timestamp_us = esp_timer_get_time();
        ESP_LOGD(TAG, "  ├─ Raw reading: %d mm (at %lld us)", mm, readings[i].timestamp_us);

        // Check for errors
        bool timeout_occurred = sensors_[i]->timeoutOccurred();
        bool i2c_failed = sensors_[i]->i2cFail();

        if (timeout_occurred) {
            ESP_LOGW(TAG, "  ├─ ⚠ Timeout occurred during reading!");
            readings[i].valid = false;
            readings[i].status = 1;
            readings[i].distance_cm = -1.0f;
        } else if (i2c_failed) {
            ESP_LOGE(TAG, "  ├─ ❌ I2C communication failure!");
            readings[i].valid = false;
            readings[i].status = 2;
            readings[i].distance_cm = -1.0f;
        } else if (mm == 65535) {
            ESP_LOGD(TAG, "  ├─ Invalid measurement (65535)");
            readings[i].valid = false;
            readings[i].status = 3;
            readings[i].distance_cm = -1.0f;
        } else {
            readings[i].valid = true;
            readings[i].status = 0;
            readings[i].distance_cm = mm / 10.0f;
            ESP_LOGD(TAG, "  ├─ ✓ Valid: %.1f cm", readings[i].distance_cm);
        }

        ESP_LOGD(TAG, "  └─ Status=%d, Valid=%d", readings[i].status, readings[i].valid);
    }

    xSemaphoreGive(array_mutex_);
    ESP_LOGD(TAG, "  ✓ Mutex released\n");

    return true;
}

// ============================================================================
// ToFSensorManager Implementation
// ============================================================================

ToFSensorManager::ToFSensorManager()
    : array_(nullptr),
      data_mutex_(nullptr),
      task_handle_(nullptr),
      running_(false),
      paused_(false) {

    ESP_LOGI(TAG_MGR, "┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
    ESP_LOGI(TAG_MGR, "▶ ToFSensorManager Constructor");
    ESP_LOGI(TAG_MGR, "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛");

    data_mutex_ = xSemaphoreCreateMutex();
    if (data_mutex_) {
        ESP_LOGI(TAG_MGR, "  ✓ Data mutex created");
    } else {
        ESP_LOGE(TAG_MGR, "  ❌ Failed to create data mutex!");
    }

    ESP_LOGI(TAG_MGR, "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛\n");
}

ToFSensorManager::~ToFSensorManager() {
    ESP_LOGI(TAG_MGR, "▶ ToFSensorManager Destructor");

    stop();

    if (data_mutex_) {
        ESP_LOGI(TAG_MGR, "  ├─ Deleting data mutex");
        vSemaphoreDelete(data_mutex_);
    }

    if (array_) {
        ESP_LOGI(TAG_MGR, "  ├─ Deleting sensor array");
        delete array_;
    }

    ESP_LOGI(TAG_MGR, "  └─ Destructor complete");
}

ToFSensorManager& ToFSensorManager::instance() {
    static ToFSensorManager mgr;
    return mgr;
}

bool ToFSensorManager::configure(const ToFArrayConfig& array_config,
                                 const ToFSensorConfig* sensor_configs,
                                 uint8_t num_sensors) {
    ESP_LOGI(TAG_MGR, "┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
    ESP_LOGI(TAG_MGR, "▶ ToFSensorManager::configure()");
    ESP_LOGI(TAG_MGR, "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛");

    if (array_) {
        ESP_LOGW(TAG_MGR, "  ⚠ Manager already configured, aborting");
        return false;
    }

    ESP_LOGI(TAG_MGR, "  Configuration:");
    ESP_LOGI(TAG_MGR, "  ├─ Number of sensors: %d", num_sensors);
    ESP_LOGI(TAG_MGR, "  ├─ I2C Port: %d", array_config.i2c_port);
    ESP_LOGI(TAG_MGR, "  ├─ I2C Freq: %lu Hz", (unsigned long)array_config.i2c_freq_hz);
    ESP_LOGI(TAG_MGR, "  ├─ SDA: GPIO %d", array_config.sda_gpio);
    ESP_LOGI(TAG_MGR, "  └─ SCL: GPIO %d", array_config.scl_gpio);

    // Step 1: Create array
    ESP_LOGI(TAG_MGR, "\n  [STEP 1] Creating ToFSensorArray");
    array_ = new ToFSensorArray(num_sensors);
    if (!array_) {
        ESP_LOGE(TAG_MGR, "  ├─ ❌ Failed to allocate array!");
        return false;
    }
    ESP_LOGI(TAG_MGR, "  └─ ✓ Array created");

    // Step 2: Initialize array
    ESP_LOGI(TAG_MGR, "\n  [STEP 2] Initializing Array (I2C Bus)");
    if (!array_->init(array_config)) {
        ESP_LOGE(TAG_MGR, "  ├─ ❌ Array initialization failed!");
        delete array_;
        array_ = nullptr;
        return false;
    }
    ESP_LOGI(TAG_MGR, "  └─ ✓ Array initialized");

    // Step 3: Add sensors
    ESP_LOGI(TAG_MGR, "\n  [STEP 3] Adding Individual Sensors");
    for (uint8_t i = 0; i < num_sensors; i++) {
        ESP_LOGI(TAG_MGR, "  ├─ Adding sensor %d/%d", i + 1, num_sensors);

        if (!array_->add_sensor(i, sensor_configs[i])) {
            ESP_LOGE(TAG_MGR, "  ├─ ❌ Failed to add sensor %d!", i);
            ESP_LOGE(TAG_MGR, "  └─ Cleaning up and aborting");
            delete array_;
            array_ = nullptr;
            return false;
        }

        ESP_LOGI(TAG_MGR, "  └─ ✓ Sensor %d added successfully", i);
    }

    // Step 4: Allocate readings buffer
    ESP_LOGI(TAG_MGR, "\n  [STEP 4] Allocating Readings Buffer");
    latest_readings_.resize(num_sensors);
    ESP_LOGI(TAG_MGR, "  └─ ✓ Buffer allocated for %d readings", num_sensors);

    ESP_LOGI(TAG_MGR, "\n┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
    ESP_LOGI(TAG_MGR, "✓ Manager Configuration COMPLETE");
    ESP_LOGI(TAG_MGR, "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛\n");

    return true;
}

bool ToFSensorManager::start_reading_task(uint32_t read_interval_ms, UBaseType_t priority) {
    ESP_LOGI(TAG_MGR, "┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
    ESP_LOGI(TAG_MGR, "▶ ToFSensorManager::start_reading_task()");
    ESP_LOGI(TAG_MGR, "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛");

    if (!array_) {
        ESP_LOGE(TAG_MGR, "  ❌ Array not configured!");
        return false;
    }

    if (task_handle_) {
        ESP_LOGW(TAG_MGR, "  ⚠ Task already running!");
        return false;
    }

    ESP_LOGI(TAG_MGR, "  Task Parameters:");
    ESP_LOGI(TAG_MGR, "  ├─ Read interval: %lu ms", (unsigned long)read_interval_ms);
    ESP_LOGI(TAG_MGR, "  └─ Priority: %lu", (unsigned long)priority);

    // Step 1: Start continuous mode
    ESP_LOGI(TAG_MGR, "\n  [STEP 1] Starting Continuous Mode");
    if (!array_->start_continuous()) {
        ESP_LOGE(TAG_MGR, "  ├─ ❌ Failed to start continuous mode!");
        return false;
    }
    ESP_LOGI(TAG_MGR, "  └─ ✓ Continuous mode started");

    // Step 2: Set task state
    ESP_LOGI(TAG_MGR, "\n  [STEP 2] Setting Task State");
    running_ = true;
    paused_ = false;
    ESP_LOGI(TAG_MGR, "  ├─ running_ = true");
    ESP_LOGI(TAG_MGR, "  └─ paused_ = false");

    // Step 3: Create task parameters
    ESP_LOGI(TAG_MGR, "\n  [STEP 3] Creating Task Parameters");
    TaskParams* params = new TaskParams{this, read_interval_ms};
    ESP_LOGI(TAG_MGR, "  ├─ Manager pointer: %p", this);
    ESP_LOGI(TAG_MGR, "  └─ Interval: %lu ms", (unsigned long)read_interval_ms);

    // Step 4: Create FreeRTOS task
    ESP_LOGI(TAG_MGR, "\n  [STEP 4] Creating FreeRTOS Task");
    ESP_LOGI(TAG_MGR, "  ├─ Name: 'tof_reading'");
    ESP_LOGI(TAG_MGR, "  ├─ Stack size: 4096 bytes");
    ESP_LOGI(TAG_MGR, "  ├─ Priority: %u", priority);
    ESP_LOGI(TAG_MGR, "  ├─ Calling xTaskCreate()...");

    BaseType_t result = xTaskCreate(reading_task, "tof_reading", 4096, params, priority, &task_handle_);

    if (result != pdPASS) {
        ESP_LOGE(TAG_MGR, "  ├─ ❌ xTaskCreate FAILED!");
        ESP_LOGE(TAG_MGR, "  └─ Return code: %d", result);
        delete params;
        running_ = false;
        return false;
    }

    ESP_LOGI(TAG_MGR, "  ├─ ✓ Task created successfully");
    ESP_LOGI(TAG_MGR, "  └─ Task handle: %p", task_handle_);

    ESP_LOGI(TAG_MGR, "\n┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
    ESP_LOGI(TAG_MGR, "✓ Reading Task Started");
    ESP_LOGI(TAG_MGR, "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛\n");

    return true;
}

void ToFSensorManager::reading_task(void* param) {
    auto* params = static_cast<TaskParams*>(param);
    ToFSensorManager* self = params->manager;
    uint32_t interval_ms = params->interval_ms;
    delete params;

    ESP_LOGI(TAG_MGR, "┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
    ESP_LOGI(TAG_MGR, "▶ Reading Task Started");
    ESP_LOGI(TAG_MGR, "  ├─ Manager: %p", self);
    ESP_LOGI(TAG_MGR, "  ├─ Interval: %lu ms", (unsigned long)interval_ms);
    ESP_LOGI(TAG_MGR, "  └─ Running on core: %d", xPortGetCoreID());
    ESP_LOGI(TAG_MGR, "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛\n");

    std::vector<SensorCommon::Reading> readings(self->latest_readings_.size());
    uint32_t cycle_count = 0;

    while (self->running_) {
        cycle_count++;

        if (self->paused_) {
            ESP_LOGD(TAG_MGR, "[Cycle %lu] Task paused, sleeping...", (unsigned long)cycle_count);
            vTaskDelay(pdMS_TO_TICKS(interval_ms));
            continue;
        }

        ESP_LOGD(TAG_MGR, "\n[Cycle %lu] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━", (unsigned long)cycle_count);
        ESP_LOGD(TAG_MGR, "[Cycle %lu] Reading sensors...", (unsigned long)cycle_count);

        int64_t start_time = esp_timer_get_time();

        if (self->array_->read_all(readings)) {
            int64_t read_time = esp_timer_get_time() - start_time;
            ESP_LOGD(TAG_MGR, "[Cycle %lu] ✓ Read complete in %lld us", (unsigned long)cycle_count, read_time);

            // Update latest readings (thread-safe)
            ESP_LOGD(TAG_MGR, "[Cycle %lu] Acquiring data mutex...", (unsigned long)cycle_count);
            if (xSemaphoreTake(self->data_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
                self->latest_readings_ = readings;
                xSemaphoreGive(self->data_mutex_);
                ESP_LOGD(TAG_MGR, "[Cycle %lu] ✓ Latest readings updated", (unsigned long)cycle_count);
            } else {
                ESP_LOGW(TAG_MGR, "[Cycle %lu] ⚠ Mutex timeout, skipping update", (unsigned long)cycle_count);
            }

            // Call callback if registered
            if (self->callback_) {
                ESP_LOGD(TAG_MGR, "[Cycle %lu] Calling user callback...", (unsigned long)cycle_count);
                self->callback_(readings);
                ESP_LOGD(TAG_MGR, "[Cycle %lu] ✓ Callback complete", (unsigned long)cycle_count);
            }

            // Log readings summary
            for (size_t i = 0; i < readings.size(); i++) {
                if (readings[i].valid) {
                    ESP_LOGI(TAG_MGR, "[Cycle %lu] Sensor %d: %.1f cm",
                             (unsigned long)cycle_count, i, readings[i].distance_cm);
                } else {
                    ESP_LOGW(TAG_MGR, "[Cycle %lu] Sensor %d: INVALID (status=%d)",
                             (unsigned long)cycle_count, i, readings[i].status);
                }
            }
        } else {
            ESP_LOGW(TAG_MGR, "[Cycle %lu] ⚠ Read failed!", (unsigned long)cycle_count);
        }

        ESP_LOGD(TAG_MGR, "[Cycle %lu] Sleeping for %lu ms...", (unsigned long)cycle_count, (unsigned long)interval_ms);
        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }

    ESP_LOGI(TAG_MGR, "\n┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
    ESP_LOGI(TAG_MGR, "▶ Reading Task Exiting");
    ESP_LOGI(TAG_MGR, "  └─ Total cycles: %lu", (unsigned long)cycle_count);
    ESP_LOGI(TAG_MGR, "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛\n");

    vTaskDelete(NULL);
}

bool ToFSensorManager::get_latest_readings(std::vector<SensorCommon::Reading>& readings) {
    ESP_LOGD(TAG_MGR, "▶ get_latest_readings() - Acquiring mutex...");

    if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        readings = latest_readings_;
        xSemaphoreGive(data_mutex_);

        ESP_LOGD(TAG_MGR, "  ✓ Retrieved %d readings", readings.size());
        return true;
    }

    ESP_LOGW(TAG_MGR, "  ❌ Mutex timeout!");
    return false;
}

void ToFSensorManager::pause() {
    ESP_LOGI(TAG_MGR, "▶ Pausing reading task");
    paused_ = true;
    ESP_LOGI(TAG_MGR, "  ✓ Task paused");
}

void ToFSensorManager::resume() {
    ESP_LOGI(TAG_MGR, "▶ Resuming reading task");
    paused_ = false;
    ESP_LOGI(TAG_MGR, "  ✓ Task resumed");
}

void ToFSensorManager::stop() {
    ESP_LOGI(TAG_MGR, "┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
    ESP_LOGI(TAG_MGR, "▶ Stopping Reading Task");
    ESP_LOGI(TAG_MGR, "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛");

    running_ = false;
    ESP_LOGI(TAG_MGR, "  ├─ Set running_ = false");

    if (task_handle_) {
        ESP_LOGI(TAG_MGR, "  ├─ Task handle exists, waiting for task to exit...");
        ESP_LOGI(TAG_MGR, "  ├─ Waiting 200ms for clean shutdown...");
        vTaskDelay(pdMS_TO_TICKS(200));
        task_handle_ = nullptr;
        ESP_LOGI(TAG_MGR, "  └─ ✓ Task stopped");
    } else {
        ESP_LOGI(TAG_MGR, "  └─ No active task");
    }

    ESP_LOGI(TAG_MGR, "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛\n");
}