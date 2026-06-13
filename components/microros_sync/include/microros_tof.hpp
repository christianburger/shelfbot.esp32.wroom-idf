#pragma once
#include "microros_common.hpp"
#include <std_msgs/msg/float32.h>

class TofComponent {
public:
    TofComponent();
    bool init(rcl_node_t* node, rclc_executor_t* executor);
    bool fini(rcl_node_t* node);
    void publishDistance(float distance_m);
private:
    rcl_publisher_t pub_;
    std_msgs__msg__Float32 msg_;
};