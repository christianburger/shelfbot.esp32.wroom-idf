#pragma once
#include "microros_common.hpp"
#include <std_msgs/msg/bool.h>

class LedComponent {
public:
    LedComponent();
    bool init(rcl_node_t* node, rclc_executor_t* executor);
    bool fini(rcl_node_t* node);
    void publishState(bool state);
private:
    rcl_publisher_t state_pub_;
    rcl_subscription_t led_sub_;
    std_msgs__msg__Bool state_msg_;
    std_msgs__msg__Bool led_msg_;
    static LedComponent* s_instance;

    static void ledCallback(const void* msg);
};