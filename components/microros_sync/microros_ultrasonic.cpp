#include "microros_ultrasonic.hpp"
#include <algorithm>

std_msgs__msg__MultiArrayDimension UltrasonicComponent::dim_[1] = {
    { {const_cast<char*>(""), 0, 1}, ULTRASONIC_MAX_CHANNELS, ULTRASONIC_MAX_CHANNELS }
};

UltrasonicComponent::UltrasonicComponent() {
    std_msgs__msg__Float32MultiArray__init(&msg_);
    msg_.layout.dim.data     = dim_;
    msg_.layout.dim.size     = 1;
    msg_.data.data           = data_;
    msg_.data.capacity       = ULTRASONIC_MAX_CHANNELS;
    msg_.data.size           = 0;
    pub_ = rcl_get_zero_initialized_publisher();
}

bool UltrasonicComponent::init(rcl_node_t* node, rclc_executor_t* executor) {
    (void)executor;   // no subscriptions or timers needed
    pub_ = rcl_get_zero_initialized_publisher();

    rcl_ret_t r = rclc_publisher_init_best_effort(&pub_, node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "shelfbot_firmware/distance_sensors");
    if (r != RCL_RET_OK) {
        ESP_LOGE("UltrasonicComponent", "pub init failed: %ld", (long)r);
        return false;
    }
    return true;
}

bool UltrasonicComponent::fini(rcl_node_t* node) {
    rcl_ret_t r = rcl_publisher_fini(&pub_, node);
    if (r != RCL_RET_OK) {
        ESP_LOGE("UltrasonicComponent", "pub fini failed: %ld", (long)r);
        return false;
    }
    pub_ = rcl_get_zero_initialized_publisher();
    return true;
}

void UltrasonicComponent::publishDistances(const float* distances, size_t count) {
    const size_t n = std::min(count, static_cast<size_t>(ULTRASONIC_MAX_CHANNELS));
    for (size_t i = 0; i < n; ++i) data_[i] = distances[i];
    msg_.data.size = n;
    publish_or_fail(&pub_, &msg_, "distance_sensors");
}
