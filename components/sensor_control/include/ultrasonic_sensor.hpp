// [file name]: ultrasonic_sensor.hpp
#pragma once

#ifndef SHELFBOT_ULTRASONIC_SENSOR_H
#define SHELFBOT_ULTRASONIC_SENSOR_H

#include "idf_c_includes.hpp"
#include "sensor_common.hpp"

// Forward declaration
class UltrasonicSensorArray;
class UltrasonicSensorManager;

// Queue for sending sensor data to the ROS publisher
extern QueueHandle_t distance_data_queue;

// ============================================================================
// Configuration Structures
// ============================================================================

typedef enum {
    SENSOR_IDLE,
    SENSOR_TRIGGERED,
    SENSOR_MEASURING
} sensor_state_t;

struct UltrasonicSensorConfig {
    gpio_num_t trig_pin;
    gpio_num_t echo_pin;
    float collision_threshold_cm = 20.0f;  // Default threshold
    uint32_t timeout_us = 30000;           // Default 30ms (~5m)
    volatile sensor_state_t state = SENSOR_IDLE;
    volatile int64_t start_time = 0;
    volatile uint32_t pulse_duration = 0; // in us
};

// ============================================================================
// UltrasonicSensorArray
// ============================================================================

class UltrasonicSensorArray {
public:
    UltrasonicSensorConfig configs_[SensorCommon::NUM_ULTRASONIC_SENSORS];
    explicit UltrasonicSensorArray(uint8_t num_sensors = SensorCommon::NUM_ULTRASONIC_SENSORS);
    ~UltrasonicSensorArray();

    bool init();
    bool add_sensor(uint8_t index, const UltrasonicSensorConfig& sensor_config);
    bool read_all_single(std::vector<SensorCommon::Reading>& readings, uint32_t timeout_ms = SensorCommon::DEFAULT_TIMEOUT_MS);

private:
    uint8_t num_sensors_;
    bool initialized_;
    SemaphoreHandle_t array_mutex_;
};

// ============================================================================
// UltrasonicSensorManager
// ============================================================================

class UltrasonicSensorManager {
public:
    static UltrasonicSensorManager& instance();

    bool configure(const UltrasonicSensorConfig* sensor_configs,
                   uint8_t num_sensors);

    bool start_reading_task(uint32_t read_interval_ms,
                            UBaseType_t priority);

    bool get_latest_readings(std::vector<SensorCommon::Reading>& readings);

    void pause();
    void resume();
    void stop();

private:
    UltrasonicSensorManager();
    ~UltrasonicSensorManager();

    static void reading_task(void* param);

    struct TaskParams {
        UltrasonicSensorManager* manager;
        uint32_t interval_ms;
    };

private:
    UltrasonicSensorArray* array_;
    std::vector<SensorCommon::Reading> latest_readings_;
    SemaphoreHandle_t data_mutex_;
    TaskHandle_t task_handle_;
    bool running_;
    bool paused_;
    std::function<void(const std::vector<SensorCommon::Reading>&)> callback_;
};

#endif // SHELFBOT_ULTRASONIC_SENSOR_H