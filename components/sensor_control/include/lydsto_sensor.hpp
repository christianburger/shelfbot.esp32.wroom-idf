#pragma once

#include <memory>
#include <vector>
#include "lydsto_lidar.hpp"
#include "sensor_common.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

struct LydstoSensorConfig {
    LydstoLidar::Config lidar_config;
};

class LydstoSensorArray {
public:
    explicit LydstoSensorArray(uint8_t num_sensors);
    ~LydstoSensorArray() = default;

    bool add_sensor(uint8_t index, const LydstoSensorConfig& cfg);
    bool update_readings(std::vector<SensorCommon::Reading>& readings);

private:
    std::vector<std::unique_ptr<LydstoLidar>> sensors_;
    uint8_t num_sensors_;
};

class LydstoSensorManager {
public:
    static LydstoSensorManager& instance();

    bool configure(const LydstoSensorConfig* sensor_configs, uint8_t num_sensors);
    bool start_reading_task(uint32_t read_interval_ms, UBaseType_t priority);
    bool get_latest_readings(std::vector<SensorCommon::Reading>& readings);

    void pause();
    void resume();
    void stop();

private:
    LydstoSensorManager();
    ~LydstoSensorManager();

    LydstoSensorManager(const LydstoSensorManager&) = delete;
    LydstoSensorManager& operator=(const LydstoSensorManager&) = delete;

    struct TaskParams {
        LydstoSensorManager* manager;
        uint32_t interval_ms;
    };

    static void reading_task(void* param);

    std::unique_ptr<LydstoSensorArray> array_;
    std::vector<SensorCommon::Reading> latest_readings_;
    SemaphoreHandle_t data_mutex_;
    TaskHandle_t task_handle_;
    bool running_;
    bool paused_;
};
