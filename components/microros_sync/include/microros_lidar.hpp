#pragma once
#include "microros_common.hpp"
#include <lidar_scan.hpp>
#include <sensor_msgs/msg/laser_scan.h>

// Number of angular buckets in the published LaserScan.
// 360 → 1° resolution, serialised size ≈ 2940 bytes (fits in RMW_UXRCE_MAX_MESSAGE_SIZE=4096).
// If publish failures recur with a smaller MAX_MESSAGE_SIZE, drop to 180.
static constexpr int LIDAR_BUCKET_COUNT = 360;

class LidarComponent {
public:
    LidarComponent();
    bool init(rcl_node_t* node, rclc_executor_t* executor);
    bool fini(rcl_node_t* node);

    /**
     * @brief Publish a completed 360° scan as a ROS LaserScan message.
     *
     * Maps per-point arrays into LIDAR_BUCKET_COUNT angular buckets using a
     * closest-wins strategy.  Logs diagnostic information every 10 scans.
     *
     * @param scan  A completed LidarScan (scan.complete must be true).
     */
    void publishLidarScan(const LidarScan& scan);

private:
    rcl_publisher_t              laser_pub_;
    sensor_msgs__msg__LaserScan  laser_msg_;
    float                        ranges_[LIDAR_BUCKET_COUNT];
    float                        intensities_[LIDAR_BUCKET_COUNT];

    // Diagnostic counters – live in the .cpp as static locals so the header
    // can be dropped in as a replacement for the original without changing
    // anything else.  Declared here only as documentation.
    // (actual storage: static locals in microros_lidar.cpp)

    void resetBuckets();
};
