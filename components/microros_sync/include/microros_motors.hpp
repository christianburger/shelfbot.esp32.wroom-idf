#pragma once
#include "microros_common.hpp"
#include <motor_control.hpp>
#include <std_msgs/msg/float32_multi_array.h>

class MotorComponent {
public:
    MotorComponent();
    bool init(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor);
    bool fini(rcl_node_t* node);
    void publishPositions(const float* positions, size_t count);
private:
    rcl_publisher_t pos_pub_;
    rcl_subscription_t cmd_sub_;
    rcl_subscription_t spd_sub_;
    rcl_timer_t pos_timer_;
    std_msgs__msg__Float32MultiArray pos_msg_;
    std_msgs__msg__Float32MultiArray cmd_msg_;
    std_msgs__msg__Float32MultiArray spd_msg_;
    float positions_[NUM_MOTORS];
    float cmd_data_[NUM_MOTORS];
    float spd_data_[NUM_MOTORS];
    static std_msgs__msg__MultiArrayDimension dim_[1];
    static MotorComponent* s_instance;

    static void commandCallback(const void* msg);
    static void speedCallback(const void* msg);
    static void positionTimerCallback(rcl_timer_t* timer, int64_t last_call_time);
};