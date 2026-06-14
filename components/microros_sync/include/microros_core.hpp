#pragma once
#include "microros_common.hpp"
#include "microros_led.hpp"
#include "microros_motors.hpp"
#include "microros_lidar.hpp"

struct MicrorosCore {
    rcl_node_t          node;
    rcl_allocator_t     allocator;
    rclc_support_t      support;
    rclc_executor_t     executor;
    bool                executor_inited;   // track executor initialization
    SemaphoreHandle_t   mutex;
    TaskHandle_t        task_handle;

    LedComponent        led;
    MotorComponent      motors;
    LidarComponent      lidar;

    rcl_publisher_t     heartbeat_pub;
    rcl_timer_t         heartbeat_timer;
    std_msgs__msg__Int32 heartbeat_msg;

    uint32_t pub_fail_count;

    MicrorosCore();
    ~MicrorosCore();

    bool initEntities();
    void destroyEntities();
};