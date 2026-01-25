// tof_sensor.hpp - Refactored with comprehensive debugging
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "vl53l0x.hpp"
#include "sensor_common.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <vector>
#include <memory>
#include <functional>

#define NUM_TOF_SENSORS 1

// ============================================================================
// Configuration Structures
// ============================================================================

struct ToFArrayConfig {
    i2c_port_t i2c_port;
    uint32_t i2c_freq_hz;
    gpio_num_t sda_gpio;
    gpio_num_t scl_gpio;
    bool enable_pullups;
    uint8_t num_sensors;
};

struct ToFSensorConfig {
    i2c_port_t i2c_port;
    uint8_t i2c_address;
    gpio_num_t xshut_gpio;
    gpio_num_t int_gpio;
    uint16_t range_mm;
    uint32_t timing_budget_ms;
};

// ============================================================================
// ToFSensorArray - Manages multiple VL53L0X sensors
// ============================================================================

class ToFSensorArray {
public:
    explicit ToFSensorArray(uint8_t num_sensors = NUM_TOF_SENSORS);
    ~ToFSensorArray();

    // Initialize I2C bus (only once for all sensors)
    bool init(const ToFArrayConfig& array_config);

    // Add individual sensor to the array
    bool add_sensor(uint8_t index, const ToFSensorConfig& sensor_config);

    // Start continuous ranging mode for all sensors
    bool start_continuous();

    // Read from all sensors
    bool read_all(std::vector<SensorCommon::Reading>& readings, uint32_t timeout_ms = SensorCommon::DEFAULT_TIMEOUT_MS);

private:
    std::vector<std::shared_ptr<VL53L0X>> sensors_;
    uint8_t num_sensors_;
    bool initialized_;
    bool i2c_installed_;
    i2c_port_t i2c_port_;
    SemaphoreHandle_t array_mutex_;
};

// ============================================================================
// ToFSensorManager - Singleton manager with background task
// ============================================================================

class ToFSensorManager {
public:
    static ToFSensorManager& instance();

    // Configure the manager with array and sensor configs
    bool configure(const ToFArrayConfig& array_config,
                   const ToFSensorConfig* sensor_configs,
                   uint8_t num_sensors);

    // Start background reading task
    bool start_reading_task(uint32_t read_interval_ms, UBaseType_t priority);

    // Get the latest readings (thread-safe)
    bool get_latest_readings(std::vector<SensorCommon::Reading>& readings);

    // Control the reading task
    void pause();
    void resume();
    void stop();

    // Optional callback for each reading cycle
    void set_callback(std::function<void(const std::vector<SensorCommon::Reading>&)> cb) {
        callback_ = cb;
    }

private:
    ToFSensorManager();
    ~ToFSensorManager();

    // Prevent copying
    ToFSensorManager(const ToFSensorManager&) = delete;
    ToFSensorManager& operator=(const ToFSensorManager&) = delete;

    static void reading_task(void* param);

    struct TaskParams {
        ToFSensorManager* manager;
        uint32_t interval_ms;
    };

private:
    ToFSensorArray* array_;
    std::vector<SensorCommon::Reading> latest_readings_;
    SemaphoreHandle_t data_mutex_;
    TaskHandle_t task_handle_;
    bool running_;
    bool paused_;
    std::function<void(const std::vector<SensorCommon::Reading>&)> callback_;
};