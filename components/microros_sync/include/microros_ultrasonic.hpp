#pragma once
#include "microros_common.hpp"
#include <sensor_common.hpp>
#include <std_msgs/msg/float32_multi_array.h>

class UltrasonicComponent {
public:
    UltrasonicComponent();
    bool init(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor);
    bool fini(rcl_node_t* node);
    void publishDistances(const float* distances, size_t count);
private:
    rcl_publisher_t pub_;
    rcl_timer_t timer_;
    std_msgs__msg__Float32MultiArray msg_;
    float data_[SensorCommon::NUM_SENSORS];
    static std_msgs__msg__MultiArrayDimension dim_[1];
    static UltrasonicComponent* s_instance;

    static void timerCallback(rcl_timer_t* timer, int64_t last_call_time);
};