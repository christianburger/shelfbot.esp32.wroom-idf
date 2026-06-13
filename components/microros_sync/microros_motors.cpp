#include "microros_motors.hpp"
#include <algorithm>

std_msgs__msg__MultiArrayDimension MotorComponent::dim_[1] = {
    { {const_cast<char*>(""), 0, 1}, NUM_MOTORS, NUM_MOTORS }
};
MotorComponent* MotorComponent::s_instance = nullptr;

void MotorComponent::commandCallback(const void* msg) {
    const auto* cmd = static_cast<const std_msgs__msg__Float32MultiArray*>(msg);
    size_t n = std::min((size_t)NUM_MOTORS, cmd->data.size);
    for (size_t i = 0; i < n; ++i)
        motor_control_set_position(i, cmd->data.data[i]);
}

void MotorComponent::speedCallback(const void* msg) {
    const auto* spd = static_cast<const std_msgs__msg__Float32MultiArray*>(msg);
    size_t n = std::min((size_t)NUM_MOTORS, spd->data.size);
    for (size_t i = 0; i < n; ++i)
        motor_control_set_velocity(i, spd->data.data[i]);
}

void MotorComponent::positionTimerCallback(rcl_timer_t*, int64_t) {
    if (!isMicrorosConnected() || !isTimeSynced()) return;
    if (lockCore() && s_instance) {
        for (uint8_t i = 0; i < NUM_MOTORS; ++i)
            s_instance->positions_[i] = motor_control_get_position(i);
        s_instance->pos_msg_.data.size = NUM_MOTORS;
        publish_or_fail(&s_instance->pos_pub_, &s_instance->pos_msg_, "motor_positions");
        unlockCore();
    }
}

MotorComponent::MotorComponent() {
    std_msgs__msg__Float32MultiArray__init(&pos_msg_);
    std_msgs__msg__Float32MultiArray__init(&cmd_msg_);
    std_msgs__msg__Float32MultiArray__init(&spd_msg_);
    pos_msg_.layout.dim.data = dim_;
    pos_msg_.layout.dim.size = 1;
    pos_msg_.data.data = positions_;
    pos_msg_.data.capacity = NUM_MOTORS;
    cmd_msg_.data.data = cmd_data_;
    cmd_msg_.data.capacity = NUM_MOTORS;
    spd_msg_.data.data = spd_data_;
    spd_msg_.data.capacity = NUM_MOTORS;

    pos_pub_ = rcl_get_zero_initialized_publisher();
    cmd_sub_ = rcl_get_zero_initialized_subscription();
    spd_sub_ = rcl_get_zero_initialized_subscription();
    pos_timer_ = rcl_get_zero_initialized_timer();
}

bool MotorComponent::init(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor) {
    s_instance = this;
    rcl_ret_t r;

    r = rclc_publisher_init_best_effort(&pos_pub_, node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "shelfbot_firmware/motor_positions");
    if (r != RCL_RET_OK) {
        ESP_LOGE("MotorComponent", "pos_pub init failed: %ld", (long)r);
        return false;
    }

    r = rclc_subscription_init_default(&cmd_sub_, node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "shelfbot_firmware/motor_command");
    if (r != RCL_RET_OK) {
        ESP_LOGE("MotorComponent", "cmd_sub init failed: %ld", (long)r);
        return false;
    }

    r = rclc_subscription_init_default(&spd_sub_, node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "shelfbot_firmware/set_speed");
    if (r != RCL_RET_OK) {
        ESP_LOGE("MotorComponent", "spd_sub init failed: %ld", (long)r);
        return false;
    }

    r = rclc_timer_init_default(&pos_timer_, support, RCL_MS_TO_NS(100),
            positionTimerCallback);
    if (r != RCL_RET_OK) {
        ESP_LOGE("MotorComponent", "pos_timer init failed: %ld", (long)r);
        return false;
    }

    if (rclc_executor_add_timer(executor, &pos_timer_) != RCL_RET_OK) {
        ESP_LOGE("MotorComponent", "add_timer failed");
        return false;
    }
    if (rclc_executor_add_subscription(executor, &cmd_sub_, &cmd_msg_,
            commandCallback, ON_NEW_DATA) != RCL_RET_OK) {
        ESP_LOGE("MotorComponent", "add_subscription(cmd) failed");
        return false;
    }
    if (rclc_executor_add_subscription(executor, &spd_sub_, &spd_msg_,
            speedCallback, ON_NEW_DATA) != RCL_RET_OK) {
        ESP_LOGE("MotorComponent", "add_subscription(spd) failed");
        return false;
    }

    return true;
}

bool MotorComponent::fini(rcl_node_t* node) {
    bool ok = true;
    rcl_ret_t r;

    r = rcl_publisher_fini(&pos_pub_, node);
    if (r != RCL_RET_OK) { ESP_LOGE("MotorComponent", "pos_pub fini failed: %ld", (long)r); ok = false; }
    r = rcl_subscription_fini(&cmd_sub_, node);
    if (r != RCL_RET_OK) { ESP_LOGE("MotorComponent", "cmd_sub fini failed: %ld", (long)r); ok = false; }
    r = rcl_subscription_fini(&spd_sub_, node);
    if (r != RCL_RET_OK) { ESP_LOGE("MotorComponent", "spd_sub fini failed: %ld", (long)r); ok = false; }
    r = rcl_timer_fini(&pos_timer_);
    if (r != RCL_RET_OK) { ESP_LOGE("MotorComponent", "pos_timer fini failed: %ld", (long)r); ok = false; }

    s_instance = nullptr;
    return ok;
}

void MotorComponent::publishPositions(const float* positions, size_t count) {
    size_t n = std::min(count, (size_t)NUM_MOTORS);
    for (size_t i = 0; i < n; ++i) positions_[i] = positions[i];
    pos_msg_.data.size = n;
    publish_or_fail(&pos_pub_, &pos_msg_, "motor_positions");
}