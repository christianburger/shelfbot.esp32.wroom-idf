#include "microros_lidar.hpp"
#include <cmath>
#include <limits>

LidarComponent::LidarComponent() {
    sensor_msgs__msg__LaserScan__init(&laser_msg_);

    // Bind the static arrays into the message. capacity/size must be consistent
    // with the 360-bucket convention used throughout this file.
    laser_msg_.ranges.data          = ranges_;
    laser_msg_.ranges.capacity      = 360;
    laser_msg_.ranges.size          = 360;
    laser_msg_.intensities.data     = intensities_;
    laser_msg_.intensities.capacity = 360;
    laser_msg_.intensities.size     = 360;

    // Physical limits for the LYDSTO LDS02RR
    laser_msg_.range_min = 0.02f;   // 2 cm
    laser_msg_.range_max = 12.0f;   // 12 m

    // 360 buckets of exactly 1° each.
    // angle_min/max follow ROS convention: radians, CCW positive.
    // angle_max = angle_min + (n-1) * angle_increment  (REP-117)
    laser_msg_.angle_min       = 0.0f;
    laser_msg_.angle_increment = static_cast<float>(M_PI) / 180.0f;          // 1° in rad
    laser_msg_.angle_max       = 359.0f * laser_msg_.angle_increment;         // 359° in rad

    // time_increment and scan_time are filled per-scan in publishLidarScan().
    laser_msg_.time_increment = 0.0f;
    laser_msg_.scan_time      = 0.0f;

    // frame_id — static storage, safe for the lifetime of the node.
    // rosidl_runtime_c__String layout: { char* data, size_t size, size_t capacity }
    // size = string length (no NUL), capacity = allocated bytes (includes NUL).
    static char frame_id[] = "laser_link";
    laser_msg_.header.frame_id = {
        frame_id,
        sizeof(frame_id) - 1,   // length without NUL  → 10
        sizeof(frame_id)         // allocated bytes      → 11
    };

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
    (void)executor;   // LaserScan is publish-only; no subscription or timer here

    laser_pub_ = rcl_get_zero_initialized_publisher();
    resetBuckets();

    // BEST_EFFORT QoS: laser scans are high-rate and the latest scan is always
    // more useful than a stale one. Matches the RViz / Nav2 default subscriber QoS.
    const rcl_ret_t r = rclc_publisher_init_best_effort(
        &laser_pub_, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "shelfbot_firmware/laser_scan");
    if (r != RCL_RET_OK) {
        ESP_LOGE("LidarComponent", "laser_pub init failed: %ld", (long)r);
        return false;
    }
    return true;
}

bool LidarComponent::fini(rcl_node_t* node) {
    const rcl_ret_t r = rcl_publisher_fini(&laser_pub_, node);
    if (r != RCL_RET_OK) {
        ESP_LOGE("LidarComponent", "laser_pub fini failed: %ld", (long)r);
        return false;
    }
    laser_pub_ = rcl_get_zero_initialized_publisher();
    return true;
}

void LidarComponent::publishLidarScan(const LidarScan& scan) {
    if (!scan.complete || scan.point_count == 0) return;

    // ── Timing fields ──────────────────────────────────────────────────────
    // scan_time: duration of one full revolution in seconds.
    // time_increment: time between adjacent angle samples (scan_time / 360).
    // Both are derived from the monotonic timestamps captured by the read task.
    if (scan.end_time_us > scan.start_time_us) {
        const float scan_time_s =
            static_cast<float>(scan.end_time_us - scan.start_time_us) * 1e-6f;
        laser_msg_.scan_time      = scan_time_s;
        laser_msg_.time_increment = scan_time_s / 360.0f;
    }

    // ── Bucket fill ────────────────────────────────────────────────────────
    // Map every point into its 1°-wide integer bucket [0, 359].
    // Keep the closest valid reading when multiple points share a bucket.
    resetBuckets();

    for (uint16_t i = 0; i < scan.point_count; ++i) {
        const float angle_deg = scan.angles_deg[i];
        if (angle_deg < 0.0f || angle_deg >= 360.0f) continue;

        const int bucket = static_cast<int>(angle_deg) % 360;

        // Per REP-117: zero distance means "no return"; report as infinity.
        // Also clamp at range_max (12 m = 12 000 mm).
        const float r = (scan.distances_mm[i] == 0 ||
                         scan.distances_mm[i] > 12000)
                        ? std::numeric_limits<float>::infinity()
                        : static_cast<float>(scan.distances_mm[i]) / 1000.0f;

        // Closest-wins: only update if this reading is better than what is
        // already in the bucket (infinity at start, so first write always wins).
        if (r < ranges_[bucket]) {
            ranges_[bucket]      = r;
            intensities_[bucket] = static_cast<float>(scan.confidences[i]);
        }
    }

    // ── Header stamp ───────────────────────────────────────────────────────
    fillRosStamp(laser_msg_.header.stamp.sec, laser_msg_.header.stamp.nanosec);

    publish_or_fail(&laser_pub_, &laser_msg_, "laser_scan");
}
