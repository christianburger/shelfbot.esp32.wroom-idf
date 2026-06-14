#include "microros_lidar.hpp"
#include <cmath>
#include <limits>

LidarComponent::LidarComponent() {
    sensor_msgs__msg__LaserScan__init(&laser_msg_);
    laser_msg_.ranges.data          = ranges_;
    laser_msg_.ranges.capacity      = 360;
    laser_msg_.ranges.size          = 360;
    laser_msg_.intensities.data     = intensities_;
    laser_msg_.intensities.capacity = 360;
    laser_msg_.intensities.size     = 360;
    laser_msg_.range_min            = 0.02f;
    laser_msg_.range_max            = 12.0f;
    laser_msg_.angle_min            = 0.0f;
    laser_msg_.angle_max            = 359.0f * static_cast<float>(M_PI) / 180.0f;
    laser_msg_.angle_increment      = static_cast<float>(M_PI) / 180.0f;
    static char frame_id[]          = "laser_link";
    laser_msg_.header.frame_id      = { frame_id, sizeof(frame_id) - 1, sizeof(frame_id) };
    laser_pub_ = rcl_get_zero_initialized_publisher();
    resetBuckets();
}

void LidarComponent::resetBuckets() {
    for (int i = 0; i < 360; ++i) {
        ranges_[i]      = std::numeric_limits<float>::infinity();
        intensities_[i] = 0.0f;
    }
}

bool LidarComponent::init(rcl_node_t* node, rclc_executor_t* executor) {
    (void)executor;
    laser_pub_ = rcl_get_zero_initialized_publisher();
    resetBuckets();

    rcl_ret_t r = rclc_publisher_init_best_effort(&laser_pub_, node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
            "shelfbot_firmware/laser_scan");
    if (r != RCL_RET_OK) {
        ESP_LOGE("LidarComponent", "laser_pub init failed: %ld", (long)r);
        return false;
    }
    return true;
}

bool LidarComponent::fini(rcl_node_t* node) {
    rcl_ret_t r = rcl_publisher_fini(&laser_pub_, node);
    if (r != RCL_RET_OK) {
        ESP_LOGE("LidarComponent", "laser_pub fini failed: %ld", (long)r);
        return false;
    }
    laser_pub_ = rcl_get_zero_initialized_publisher();
    return true;
}

void LidarComponent::publishLidarScan(const LidarScan& scan) {
    if (!scan.complete || scan.point_count == 0) return;

    // Map every point into its 1°-wide bucket.
    // Keep the closest reading when multiple points fall in the same bucket.
    resetBuckets();

    for (uint16_t i = 0; i < scan.point_count; ++i) {
        const float angle = scan.angles_deg[i];
        if (angle < 0.0f || angle >= 360.0f) continue;

        const int bucket = static_cast<int>(angle) % 360;

        // Zero distance means "no return"; treat as infinity
        const float r = (scan.distances_mm[i] == 0 || scan.distances_mm[i] > 12000)
                        ? std::numeric_limits<float>::infinity()
                        : scan.distances_mm[i] / 1000.0f;

        if (r < ranges_[bucket]) {
            ranges_[bucket]      = r;
            intensities_[bucket] = static_cast<float>(scan.confidences[i]);
        }
    }

    fillRosStamp(laser_msg_.header.stamp.sec, laser_msg_.header.stamp.nanosec);
    publish_or_fail(&laser_pub_, &laser_msg_, "laser_scan");
}
