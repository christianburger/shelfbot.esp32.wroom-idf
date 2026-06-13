#pragma once
#include "microros_common.hpp"
#include <sensor_common.hpp>
#include <sensor_msgs/msg/laser_scan.h>

class LidarComponent {
public:
    LidarComponent();
    bool init(rcl_node_t* node, rclc_executor_t* executor);
    bool fini(rcl_node_t* node);
    void publishLidarScan(const SensorCommon::LidarMeasurement& m);
private:
    rcl_publisher_t laser_pub_;
    sensor_msgs__msg__LaserScan laser_msg_;
    float ranges_[360];
    float intensities_[360];
    float last_end_angle_deg_;
    void resetBuckets();
};