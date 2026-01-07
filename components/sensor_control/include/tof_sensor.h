// Updated tof_sensor.h with interrupt support
#pragma once

#include <cstdint>
#include <functional>
#include <vector>

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "sensor_common.h"  // New include for shared types

// ============================================================================
// Constants (MUST match cpp name)
// ============================================================================

namespace ToFConstants {

    constexpr uint8_t DEFAULT_ADDRESS = 0x29;
    constexpr uint8_t ADDRESS_MIN     = 0x08;
    constexpr uint8_t ADDRESS_MAX     = 0x77;

    constexpr uint16_t MIN_TIMING_BUDGET_MS = 20;
    constexpr uint16_t MAX_TIMING_BUDGET_MS = 200;
}

// ============================================================================
// Data Types (Map to common Reading)
// ============================================================================

using Reading = SensorCommon::Reading;  // Alias for unified reading

// ============================================================================
// Configuration Structures
// ============================================================================

struct ToFSensorConfig {
    i2c_port_t i2c_port;
    uint8_t    i2c_address;
    uint8_t    xshut_gpio;        // 255 if unused
    gpio_num_t int_gpio = GPIO_NUM_MAX;  // Interrupt pin, GPIO_NUM_MAX if polling
    uint16_t   range_mm;
    uint16_t   timing_budget_ms;
    bool       interrupt_active_high = false;  // Default active low
};

struct ToFArrayConfig {
    i2c_port_t i2c_port;
    uint32_t   i2c_freq_hz;
    int        sda_gpio;
    int        scl_gpio;
    bool       enable_pullups;
    uint8_t    num_sensors;
};

// ============================================================================
// ToFSensor
// ============================================================================

class ToFSensor {
public:
    explicit ToFSensor(const ToFSensorConfig& config);
    ~ToFSensor();

    bool init();
    bool read_single(Reading& reading, uint32_t timeout_ms = SensorCommon::DEFAULT_TIMEOUT_MS);

    bool start_continuous();
    bool stop_continuous();

    bool set_address(uint8_t new_address);
    uint8_t get_address() const;

private:
    bool write_register(uint8_t reg, uint8_t value);
    bool write_register_16(uint8_t reg, uint16_t value);
    bool read_register(uint8_t reg, uint8_t& value);
    bool read_register_16(uint8_t reg, uint16_t& value);

    bool set_signal_rate_limit(float limit_mcps);
    bool get_spad_info(uint8_t& count, bool& type_is_aperture);
    bool set_timing_budget(uint16_t budget_ms);
    bool configure_interrupt();

    static void IRAM_ATTR interrupt_handler(void* arg);

private:
    ToFSensorConfig   config_;
    bool              initialized_;
    bool              continuous_mode_;
    SemaphoreHandle_t i2c_mutex_;
    SemaphoreHandle_t measurement_sem_;
};

// ============================================================================
// ToFSensorArray
// ============================================================================

class ToFSensorArray {
public:
    explicit ToFSensorArray(const ToFArrayConfig& config);
    ~ToFSensorArray();

    bool init();
    bool add_sensor(const ToFSensorConfig& sensor_config);
    bool read_all_single(std::vector<Reading>& readings, uint32_t timeout_ms = SensorCommon::DEFAULT_TIMEOUT_MS);

private:
    bool init_i2c_bus();
    bool program_sensor_addresses();

private:
    ToFArrayConfig    config_;
    ToFSensor**       sensors_;
    uint8_t           sensor_count_;
    bool              initialized_;
    SemaphoreHandle_t array_mutex_;
};

// ============================================================================
// ToFSensorManager (ORDER MATTERS)
// ============================================================================

class ToFSensorManager {
public:
    static ToFSensorManager& instance();

    bool configure(const ToFArrayConfig& array_config,
                   const ToFSensorConfig* sensor_configs,
                   uint8_t num_sensors);

    bool start_reading_task(uint32_t read_interval_ms,
                            UBaseType_t priority);

    bool get_latest_readings(std::vector<Reading>& readings);

    void pause();
    void resume();
    void stop();

private:
    ToFSensorManager();
    ~ToFSensorManager();

    static void reading_task(void* param);

    struct TaskParams {
        ToFSensorManager* manager;
        uint32_t interval_ms;
    };

private:
    // ORDER MATCHES CONSTRUCTOR INITIALIZER LIST
    ToFSensorArray*   array_;
    std::vector<Reading> latest_readings_;  // Use vector for dynamic size
    SemaphoreHandle_t data_mutex_;
    TaskHandle_t      task_handle_;
    bool              running_;
    bool              paused_;
    std::function<void(const std::vector<Reading>&)> callback_;
};