#pragma once
#include <idf_c_includes.hpp>
#include <lidar_scan.hpp>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "microros_led.hpp"
#include "microros_motors.hpp"
#include "microros_lidar.hpp"

class MicrorosSync {
public:
    static MicrorosSync& getInstance();

    // Registers prerequisites with the state machine.
    // Does NOT create a task — call microros_task_fn via shelfbot.cpp.
    bool init();

    // Task function — created by shelfbot.cpp.
    // Stack budget: 12288 words (48 KB).
    //   RCL/XRCE context (~16 KB) + executor spin buffer (~4 KB)
    //   + LidarScan static local (~14 KB, declared static so not on stack)
    //   + FreeRTOS overhead + nested call frames.
    //   24576 (original 96 KB) was catastrophically oversized for a no-PSRAM
    //   ESP32.  12288 gives comfortable headroom while recovering ~36 KB of heap.
    static void microros_task_fn(void* arg);

    bool createEntities();
    void destroyEntities();

    bool isConnected() const;

    static void publishHeartbeat(int32_t value);
    static void publishMotorPositions(const float* positions, size_t count);
    static void publishDistanceSensors(const float* distances, size_t count);
    static void publishLedState(bool state);
    static void publishTofDistance(float distance_m);
    static void publishLidarScan(const LidarScan& scan);

    static bool lockCore();
    static void unlockCore();

private:
    MicrorosSync();
    ~MicrorosSync();

    rcl_node_t node_;
    rcl_allocator_t allocator_;
    rclc_support_t support_;
    rclc_executor_t executor_;
    bool entities_created_;
    bool support_inited_;

    LedComponent        led_;
    MotorComponent      motors_;
    LidarComponent      lidar_;

    rcl_publisher_t      heartbeat_pub_;
    rcl_timer_t          heartbeat_timer_;
    std_msgs__msg__Int32 heartbeat_msg_;

    rcl_timer_t          lidar_timer_;

    enum ComponentId : uint8_t {
        COMP_LED,
        COMP_MOTORS,
        COMP_LIDAR,
        COMP_COUNT
    };
    bool comp_initialized_[COMP_COUNT];

    static MicrorosSync*     instance_;
    static SemaphoreHandle_t mutex_;

    static void heartbeatTimerCallback(rcl_timer_t* timer, int64_t last_call_time);
    static void lidarTimerCallback(rcl_timer_t* timer, int64_t last_call_time);

    bool createEntitiesImpl();
    void destroyEntitiesImpl();
    bool queryAgentIp(char* out_ip, size_t len);
    void logMicrorosLimits();
};
