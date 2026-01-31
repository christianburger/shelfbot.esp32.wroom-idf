#pragma once

#include <cstdint>
#include <memory>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "sensor_control.hpp"      // SensorControl class
#include "ultrasonic_sensor.hpp"   // NUM_ULTRASONIC_SENSORS

// ---------------------------------------------------------------------------
// Constants  (the only place these are defined)
// ---------------------------------------------------------------------------
static constexpr int NUM_TOF_SENSORS = 1;
static constexpr int NUM_SENSORS     = NUM_ULTRASONIC_SENSORS + NUM_TOF_SENSORS;

// ---------------------------------------------------------------------------
// Data packet — filled by the background task, read by Shelfbot's ROS timers
// ---------------------------------------------------------------------------
struct SensorDataPacket {
    float ultrasonic_distances_cm[NUM_ULTRASONIC_SENSORS]{};
    bool  ultrasonic_valid[NUM_ULTRASONIC_SENSORS]{};

    float tof_distances_cm[NUM_TOF_SENSORS]{};
    bool  tof_valid[NUM_TOF_SENSORS]{};

    bool  data_ready{ false };
};

// ---------------------------------------------------------------------------
// Singleton facade — pure C++, zero free functions, zero C linkage.
//
//     SensorControlFacade::init();
//     SensorControlFacade::start_task();
//     SensorDataPacket pkt;
//     if (SensorControlFacade::get_latest_data(pkt)) { … }
// ---------------------------------------------------------------------------
class SensorControlFacade {
public:
    static void init();
    static void start_task();
    static bool get_latest_data(SensorDataPacket& out);

private:
    SensorControlFacade() = default;
    ~SensorControlFacade() = default;
    SensorControlFacade(const SensorControlFacade&) = delete;
    SensorControlFacade& operator=(const SensorControlFacade&) = delete;

    static SensorControlFacade& instance();

    std::unique_ptr<SensorControl> sensor_control_;
    SemaphoreHandle_t              mutex_{ nullptr };
    TaskHandle_t                   task_handle_{ nullptr };
    SensorDataPacket               latest_{};

    static void task_entry(void* arg);
    void        run();
};