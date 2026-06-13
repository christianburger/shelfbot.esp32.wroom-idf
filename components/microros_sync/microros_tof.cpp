#include "microros_tof.hpp"

TofComponent::TofComponent() {
    std_msgs__msg__Float32__init(&msg_);
    pub_ = rcl_get_zero_initialized_publisher();
}

bool TofComponent::init(rcl_node_t* node, rclc_executor_t* executor) {
    (void)executor;
    rcl_ret_t r = rclc_publisher_init_best_effort(&pub_, node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "shelfbot_firmware/tof_distance");
    if (r != RCL_RET_OK) {
        ESP_LOGE("TofComponent", "pub init failed: %ld", (long)r);
        return false;
    }
    return true;
}

bool TofComponent::fini(rcl_node_t* node) {
    rcl_ret_t r = rcl_publisher_fini(&pub_, node);
    if (r != RCL_RET_OK) {
        ESP_LOGE("TofComponent", "pub fini failed: %ld", (long)r);
        return false;
    }
    return true;
}

void TofComponent::publishDistance(float distance_m) {
    msg_.data = distance_m;
    publish_or_fail(&pub_, &msg_, "tof_distance");
}