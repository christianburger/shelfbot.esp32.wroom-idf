#pragma once

#include <vector>
#include <memory>
#include "vl53l0x.hpp"
#include "sensor_common.hpp"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/**
 * @brief Configuration for a single ToF sensor
 */
struct ToFSensorConfig {
    i2c_port_t i2c_port = I2C_NUM_0;
    uint8_t i2c_address = 0x29;
    gpio_num_t sda_pin = GPIO_NUM_21;
    gpio_num_t scl_pin = GPIO_NUM_22;
    gpio_num_t xshut_pin = GPIO_NUM_NC;
    bool io_2v8 = true;
    uint16_t timeout_ms = 500;
    uint32_t timing_budget_us = 200000;
    float signal_rate_limit_mcps = 0.25;
};

/**
 * @brief Low-level array manager for multiple ToF sensors
 *
 * Manages multiple VL53L0X sensors without direct I2C access
 */
class ToFSensorArray {
public:
    explicit ToFSensorArray(uint8_t num_sensors);
    ~ToFSensorArray() = default;

    /**
     * @brief Add a sensor to the array
     * @param index Sensor index
     * @param cfg Sensor configuration
     * @return true on success, false on failure
     */
    bool add_sensor(uint8_t index, const ToFSensorConfig& cfg);

    /**
     * @brief Update readings from all sensors
     * @param readings Output vector of readings
     * @return true if any sensor provided data, false otherwise
     */
    bool update_readings(std::vector<SensorCommon::Reading>& readings);

private:
    std::vector<std::unique_ptr<VL53L0X>> sensors_;
    uint8_t num_sensors_;
};

/**
 * @brief High-level manager with task and queue for ToF sensors
 *
 * Provides automatic background reading with FreeRTOS task management
 */
class ToFSensorManager {
public:
    static ToFSensorManager& instance();

    /**
     * @brief Configure the ToF sensor manager
     * @param sensor_configs Array of sensor configurations
     * @param num_sensors Number of sensors
     * @return true on success, false on failure
     */
    bool configure(const ToFSensorConfig* sensor_configs, uint8_t num_sensors);

    /**
     * @brief Start the background reading task
     * @param read_interval_ms Reading interval in milliseconds
     * @param priority FreeRTOS task priority
     * @return true on success, false on failure
     */
    bool start_reading_task(uint32_t read_interval_ms, UBaseType_t priority);

    /**
     * @brief Get latest readings from all sensors
     * @param readings Output vector of readings
     * @return true on success, false on failure
     */
    bool get_latest_readings(std::vector<SensorCommon::Reading>& readings);

    /**
     * @brief Pause reading task
     */
    void pause();

    /**
     * @brief Resume reading task
     */
    void resume();

    /**
     * @brief Stop reading task
     */
    void stop();

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

    std::unique_ptr<ToFSensorArray> array_;
    std::vector<SensorCommon::Reading> latest_readings_;
    SemaphoreHandle_t data_mutex_;
    TaskHandle_t task_handle_;
    bool running_;
    bool paused_;
};
