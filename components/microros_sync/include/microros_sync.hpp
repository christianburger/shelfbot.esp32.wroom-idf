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

    bool init();
    void start();

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

    // ROS entities
    rcl_node_t node_;
    rcl_allocator_t allocator_;
    rclc_support_t support_;
    rclc_executor_t executor_;
    bool entities_created_;
    bool support_inited_;

    // Components
    LedComponent        led_;
    MotorComponent      motors_;
    LidarComponent      lidar_;

    // Heartbeat
    rcl_publisher_t      heartbeat_pub_;
    rcl_timer_t          heartbeat_timer_;
    std_msgs__msg__Int32 heartbeat_msg_;

    // LiDAR timer
    rcl_timer_t          lidar_timer_;   // ADDED

    // Task handle
    TaskHandle_t task_handle_;

    // Static instance for callbacks
    static MicrorosSync*    instance_;
    static SemaphoreHandle_t mutex_;

    // Component initialization tracking
    enum ComponentId : uint8_t {
        COMP_LED,
        COMP_MOTORS,
        COMP_LIDAR,
        COMP_COUNT
    };
    bool comp_initialized_[COMP_COUNT];

    // Task function
    static void microros_task(void* arg);
    static void heartbeatTimerCallback(rcl_timer_t* timer, int64_t last_call_time);
    static void lidarTimerCallback(rcl_timer_t* timer, int64_t last_call_time); // ADDED

    bool createEntitiesImpl();
    void destroyEntitiesImpl();
    bool queryAgentIp(char* out_ip, size_t len);
    void logMicrorosLimits();
};