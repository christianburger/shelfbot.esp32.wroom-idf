#pragma once

#include <memory>
#include <vector>
#include "lydsto_lidar.hpp"
#include "sensor_common.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// High-level generic LiDAR config (currently backed by Lydsto low-level driver).
struct LidarSensorConfig {
    LydstoLidar::Config driver_config;
};

class LidarSensorArray {
public:
    explicit LidarSensorArray(uint8_t num_sensors);
    ~LidarSensorArray() = default;

    bool add_sensor(uint8_t index, const LidarSensorConfig& cfg);
    bool update_readings(std::vector<SensorCommon::Reading>& readings);

private:
    std::vector<std::unique_ptr<LydstoLidar>> sensors_;
    uint8_t num_sensors_;
};

class LidarSensorManager {
public:
    static LidarSensorManager& instance();

    bool configure(const LidarSensorConfig* sensor_configs, uint8_t num_sensors);
    bool start_reading_task(uint32_t read_interval_ms, UBaseType_t priority);
    bool get_latest_readings(std::vector<SensorCommon::Reading>& readings);

    void pause();
    void resume();
    void stop();

private:
    LidarSensorManager();
    ~LidarSensorManager();

    LidarSensorManager(const LidarSensorManager&) = delete;
    LidarSensorManager& operator=(const LidarSensorManager&) = delete;

    struct TaskParams {
        LidarSensorManager* manager;
        uint32_t interval_ms;
    };

    static void reading_task(void* param);

    std::unique_ptr<LidarSensorArray> array_;
    std::vector<SensorCommon::Reading> latest_readings_;
    SemaphoreHandle_t data_mutex_;
    TaskHandle_t task_handle_;
    bool running_;
    bool paused_;
};
