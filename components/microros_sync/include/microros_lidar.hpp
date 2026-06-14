#pragma once
#include "microros_common.hpp"
#include <lidar_scan.hpp>
#include <sensor_msgs/msg/laser_scan.h>

class LidarComponent {
public:
    LidarComponent();
    bool init(rcl_node_t* node, rclc_executor_t* executor);
    bool fini(rcl_node_t* node);

    /**
     * @brief Publish a completed 360° scan as a ROS LaserScan message.
     *
     * Called once per revolution when lidar_get_latest_scan() returns true.
     * The scan's per-point arrays are mapped directly into the 360-bucket
     * range array; no intermediate LidarMeasurement struct is needed.
     *
     * @param scan  A completed LidarScan (scan.complete must be true).
     */
    void publishLidarScan(const LidarScan& scan);

private:
    rcl_publisher_t laser_pub_;
    sensor_msgs__msg__LaserScan laser_msg_;
    float ranges_[360];
    float intensities_[360];
    void resetBuckets();
};
