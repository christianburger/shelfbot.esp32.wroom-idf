#include "microros_led.hpp"
#include <led_control.hpp>

LedComponent* LedComponent::s_instance = nullptr;

LedComponent::LedComponent() {
    std_msgs__msg__Bool__init(&state_msg_);
    std_msgs__msg__Bool__init(&led_msg_);
    state_pub_ = rcl_get_zero_initialized_publisher();
    led_sub_ = rcl_get_zero_initialized_subscription();
}

void LedComponent::ledCallback(const void* msg) {
    const auto* led = static_cast<const std_msgs__msg__Bool*>(msg);
    led_control_set(led->data);
    if (isMicrorosConnected() && lockCore() && s_instance) {
        s_instance->state_msg_.data = led->data;
        publish_or_fail(&s_instance->state_pub_, &s_instance->state_msg_, "led_state");
        unlockCore();
    }
}

bool LedComponent::init(rcl_node_t* node, rclc_executor_t* executor) {
    s_instance = this;
    rcl_ret_t r;

    r = rclc_publisher_init_default(&state_pub_, node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            "shelfbot_firmware/led_state");
    if (r != RCL_RET_OK) {
        ESP_LOGE("LedComponent", "state_pub init failed: %ld", (long)r);
        return false;
    }

    r = rclc_subscription_init_default(&led_sub_, node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            "shelfbot_firmware/led");
    if (r != RCL_RET_OK) {
        ESP_LOGE("LedComponent", "led_sub init failed: %ld", (long)r);
        return false;
    }

    r = rclc_executor_add_subscription(executor, &led_sub_, &led_msg_,
            ledCallback, ON_NEW_DATA);
    if (r != RCL_RET_OK) {
        ESP_LOGE("LedComponent", "add_subscription failed: %ld", (long)r);
        return false;
    }

    return true;
}

bool LedComponent::fini(rcl_node_t* node) {
    bool ok = true;
    rcl_ret_t r;

    r = rcl_publisher_fini(&state_pub_, node);
    if (r != RCL_RET_OK) {
        ESP_LOGE("LedComponent", "state_pub fini failed: %ld", (long)r);
        ok = false;
    }
    r = rcl_subscription_fini(&led_sub_, node);
    if (r != RCL_RET_OK) {
        ESP_LOGE("LedComponent", "led_sub fini failed: %ld", (long)r);
        ok = false;
    }

    s_instance = nullptr;
    return ok;
}

void LedComponent::publishState(bool state) {
    state_msg_.data = state;
    publish_or_fail(&state_pub_, &state_msg_, "led_state");
}