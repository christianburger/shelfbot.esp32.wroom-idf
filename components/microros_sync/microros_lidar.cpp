#include "microros_lidar.hpp"
#include <cmath>
#include <limits>

LidarComponent::LidarComponent() : last_end_angle_deg_(-1.0f) {
    sensor_msgs__msg__LaserScan__init(&laser_msg_);
    laser_msg_.ranges.data = ranges_;
    laser_msg_.ranges.capacity = 360;
    laser_msg_.ranges.size = 360;
    laser_msg_.intensities.data = intensities_;
    laser_msg_.intensities.capacity = 360;
    laser_msg_.intensities.size = 360;
    laser_msg_.range_min = 0.02f;
    laser_msg_.range_max = 12.0f;
    laser_msg_.angle_min = 0.0f;
    laser_msg_.angle_max = 359.0f * static_cast<float>(M_PI) / 180.0f;
    laser_msg_.angle_increment = static_cast<float>(M_PI) / 180.0f;
    static char frame_id[] = "laser_link";
    laser_msg_.header.frame_id = { frame_id, sizeof(frame_id)-1, sizeof(frame_id) };
    resetBuckets();
    laser_pub_ = rcl_get_zero_initialized_publisher();
}

void LidarComponent::resetBuckets() {
    for (int i = 0; i < 360; ++i) {
        ranges_[i] = std::numeric_limits<float>::infinity();
        intensities_[i] = 0.0f;
    }
}

bool LidarComponent::init(rcl_node_t* node, rclc_executor_t* executor) {
    (void)executor; // no timers or subscriptions
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
    return true;
}

void LidarComponent::publishLidarScan(const SensorCommon::LidarMeasurement& m) {
    if (!m.valid || !m.has_packet_points) return;
    // Assumes lock is already held by caller (MicrorosSync::publishLidarScan)

    float span_deg = (m.end_angle_deg > m.start_angle_deg)
                     ? (m.end_angle_deg - m.start_angle_deg)
                     : (m.end_angle_deg + 360.0f - m.start_angle_deg);
    for (int i = 0; i < m.sample_count && i < 12; ++i) {
        float angle_deg = m.start_angle_deg
                        + span_deg * static_cast<float>(i)
                        / static_cast<float>(m.sample_count - 1);
        if (angle_deg >= 360.0f) angle_deg -= 360.0f;
        if (angle_deg < 0.0f)    angle_deg += 360.0f;
        int bucket = static_cast<int>(angle_deg) % 360;
        float r = (m.packet_distances_mm[i] == 0 || m.packet_distances_mm[i] > 12000)
                  ? std::numeric_limits<float>::infinity()
                  : m.packet_distances_mm[i] / 1000.0f;
        if (r < ranges_[bucket]) {
            ranges_[bucket] = r;
            intensities_[bucket] = static_cast<float>(m.packet_confidences[i]);
        }
    }

    bool revolution_complete = (last_end_angle_deg_ >= 0.0f &&
                                m.start_angle_deg < last_end_angle_deg_ - 180.0f);
    last_end_angle_deg_ = m.end_angle_deg;

    if (revolution_complete) {
        fillRosStamp(laser_msg_.header.stamp.sec, laser_msg_.header.stamp.nanosec);
        publish_or_fail(&laser_pub_, &laser_msg_, "laser_scan");
        resetBuckets();
    }
}