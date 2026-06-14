#include "microros_lidar.hpp"
#include <cmath>
#include <limits>

static const char* TAG_LIDAR_PUB = "LidarPub";

// ─────────────────────────────────────────────────────────────────────────────
//  Serialised size of a sensor_msgs/LaserScan with LIDAR_BUCKET_COUNT buckets:
//
//    header          ~24 bytes
//    7 × float32     ~28 bytes
//    ranges array     4 + LIDAR_BUCKET_COUNT × 4 bytes
//    intensities      4 + LIDAR_BUCKET_COUNT × 4 bytes
//    ──────────────────────────────────────────────────
//    360 buckets  →  ~2940 bytes  (fits in RMW_UXRCE_MAX_MESSAGE_SIZE=4096)
//    180 buckets  →  ~1500 bytes  (safe fallback if MAX_MESSAGE_SIZE is lower)
// ─────────────────────────────────────────────────────────────────────────────

// ── Intermediate accumulation grid ───────────────────────────────────────────
//
// Root cause of the alternating-inf pattern
// ─────────────────────────────────────────
// The LYDSTO sends 12 points per packet spanning ~2.5° of arc.
// Each interpolated step is  2.5° / 11 ≈ 0.23°.
// With 1°-wide output buckets, floor(angle) maps all 12 points into only
// 2-3 integer buckets, leaving the adjacent buckets permanently empty →
// the every-other-bucket .inf pattern seen in the topic echo.
//
// Fix: accumulate into a 720-slot grid (0.5° resolution) then downsample
// into the 360-slot output by taking the best (closest) reading from each
// pair of half-degree slots.  At 0.5° resolution, 0.23° steps hit different
// slots on nearly every point, so the output is fully populated.
//
// FINE_SLOTS must be an integer multiple of LIDAR_BUCKET_COUNT.
static constexpr int FINE_SLOTS = 720;   // 0.5° each

// Static diagnostic counters (not class members, so no header change needed)
static uint32_t s_pub_fail_streak = 0;
static uint32_t s_pub_ok_total    = 0;
static uint32_t s_pub_fail_total  = 0;

// Fine-resolution accumulation arrays — static to avoid stack pressure.
// Safe because publishLidarScan is called only from the executor thread
// (sequential, never re-entrant).
static float s_fine_ranges[FINE_SLOTS];
static float s_fine_intensities[FINE_SLOTS];

static void resetFine() {
    for (int i = 0; i < FINE_SLOTS; ++i) {
        s_fine_ranges[i]      = std::numeric_limits<float>::infinity();
        s_fine_intensities[i] = 0.0f;
    }
}

LidarComponent::LidarComponent() {
    sensor_msgs__msg__LaserScan__init(&laser_msg_);

    laser_msg_.ranges.data          = ranges_;
    laser_msg_.ranges.capacity      = LIDAR_BUCKET_COUNT;
    laser_msg_.ranges.size          = LIDAR_BUCKET_COUNT;
    laser_msg_.intensities.data     = intensities_;
    laser_msg_.intensities.capacity = LIDAR_BUCKET_COUNT;
    laser_msg_.intensities.size     = LIDAR_BUCKET_COUNT;

    laser_msg_.range_min = 0.02f;
    laser_msg_.range_max = 12.0f;

    // LIDAR_BUCKET_COUNT equal-width buckets covering [0, 360°).
    const float inc = static_cast<float>(2.0 * M_PI) / static_cast<float>(LIDAR_BUCKET_COUNT);
    laser_msg_.angle_min       = 0.0f;
    laser_msg_.angle_increment = inc;
    laser_msg_.angle_max       = static_cast<float>(LIDAR_BUCKET_COUNT - 1) * inc;

    laser_msg_.time_increment = 0.0f;
    laser_msg_.scan_time      = 0.0f;

    static char frame_id[] = "laser_link";
    laser_msg_.header.frame_id = {
        frame_id,
        sizeof(frame_id) - 1,
        sizeof(frame_id)
    };

    laser_pub_ = rcl_get_zero_initialized_publisher();
    resetBuckets();
}

void LidarComponent::resetBuckets() {
    for (int i = 0; i < LIDAR_BUCKET_COUNT; ++i) {
        ranges_[i]      = std::numeric_limits<float>::infinity();
        intensities_[i] = 0.0f;
    }
}

bool LidarComponent::init(rcl_node_t* node, rclc_executor_t* executor) {
    (void)executor;

    laser_pub_ = rcl_get_zero_initialized_publisher();
    s_pub_fail_streak = 0;
    s_pub_ok_total    = 0;
    s_pub_fail_total  = 0;
    resetBuckets();

    // RELIABLE QoS: matches Nav2/RViz2 default for /scan; avoids the
    // RCL_RET_ERROR (code 1) failures seen with BEST_EFFORT when the XRCE
    // session write slot is exhausted.
    const rcl_ret_t r = rclc_publisher_init_default(
        &laser_pub_, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "shelfbot_firmware/laser_scan");

    if (r != RCL_RET_OK) {
        ESP_LOGE(TAG_LIDAR_PUB, "laser_pub init FAILED: %ld", (long)r);
        return false;
    }

    ESP_LOGI(TAG_LIDAR_PUB,
             "laser_pub init OK (RELIABLE, topic=shelfbot_firmware/laser_scan, "
             "out_buckets=%d fine_slots=%d)",
             LIDAR_BUCKET_COUNT, FINE_SLOTS);
    return true;
}

bool LidarComponent::fini(rcl_node_t* node) {
    const rcl_ret_t r = rcl_publisher_fini(&laser_pub_, node);
    if (r != RCL_RET_OK) {
        ESP_LOGE(TAG_LIDAR_PUB, "laser_pub fini FAILED: %ld", (long)r);
        return false;
    }
    laser_pub_ = rcl_get_zero_initialized_publisher();
    ESP_LOGI(TAG_LIDAR_PUB, "laser_pub fini OK (session: ok=%lu fail=%lu)",
             (unsigned long)s_pub_ok_total, (unsigned long)s_pub_fail_total);
    return true;
}

void LidarComponent::publishLidarScan(const LidarScan& scan) {
    if (!scan.complete || scan.point_count == 0) {
        ESP_LOGD(TAG_LIDAR_PUB, "publishLidarScan: skip (complete=%d pts=%u)",
                 (int)scan.complete, (unsigned)scan.point_count);
        return;
    }

    // ── Timing ────────────────────────────────────────────────────────────
    float scan_time_s = 0.0f;
    if (scan.end_time_us > scan.start_time_us) {
        scan_time_s = static_cast<float>(scan.end_time_us - scan.start_time_us) * 1e-6f;
        laser_msg_.scan_time      = scan_time_s;
        laser_msg_.time_increment = scan_time_s / static_cast<float>(LIDAR_BUCKET_COUNT);
    }

    // ── Stage 1: accumulate into fine-resolution grid ─────────────────────
    //
    // Each point's angle is mapped to the nearest 0.5° slot (FINE_SLOTS=720).
    // At this resolution, consecutive LYDSTO points (~0.23° apart) almost
    // always land in different slots, so no slot is structurally skipped.
    //
    // Closest-wins within each fine slot.
    resetFine();

    uint16_t points_mapped   = 0;
    uint16_t points_invalid  = 0;
    uint16_t points_overflow = 0;

    for (uint16_t i = 0; i < scan.point_count; ++i) {
        const float angle_deg = scan.angles_deg[i];

        // Guard: angles must be in [0, 360)
        if (angle_deg < 0.0f || angle_deg >= 360.0f) {
            ++points_invalid;
            continue;
        }

        // Nearest fine slot (round, not floor)
        const int fine = static_cast<int>(
            lroundf(angle_deg * static_cast<float>(FINE_SLOTS) / 360.0f)
        ) % FINE_SLOTS;

        const uint16_t d_mm = scan.distances_mm[i];
        float r_m;
        if (d_mm == 0 || d_mm > 12000) {
            r_m = std::numeric_limits<float>::infinity();
            if (d_mm > 12000) ++points_overflow;
        } else {
            r_m = static_cast<float>(d_mm) / 1000.0f;
        }

        if (r_m < s_fine_ranges[fine]) {
            s_fine_ranges[fine]      = r_m;
            s_fine_intensities[fine] = static_cast<float>(scan.confidences[i]);
            ++points_mapped;
        }
    }

    // ── Stage 2: downsample fine grid → output buckets ────────────────────
    //
    // Each output bucket covers (FINE_SLOTS / LIDAR_BUCKET_COUNT) = 2 fine slots.
    // We take the minimum range (closest valid reading) across those fine slots.
    // If all fine slots in a bucket are inf the output is inf (genuine no-return).
    static_assert(FINE_SLOTS % LIDAR_BUCKET_COUNT == 0,
                  "FINE_SLOTS must be a multiple of LIDAR_BUCKET_COUNT");
    constexpr int RATIO = FINE_SLOTS / LIDAR_BUCKET_COUNT;  // = 2

    resetBuckets();
    int filled_buckets = 0;
    for (int b = 0; b < LIDAR_BUCKET_COUNT; ++b) {
        for (int f = 0; f < RATIO; ++f) {
            const int fine = b * RATIO + f;
            if (s_fine_ranges[fine] < ranges_[b]) {
                ranges_[b]      = s_fine_ranges[fine];
                intensities_[b] = s_fine_intensities[fine];
            }
        }
        if (std::isfinite(ranges_[b])) ++filled_buckets;
    }

    // ── Header stamp ──────────────────────────────────────────────────────
    fillRosStamp(laser_msg_.header.stamp.sec, laser_msg_.header.stamp.nanosec);

    // ── Periodic diagnostic log (every 10 scans) ──────────────────────────
    static uint32_t s_scan_seq = 0;
    ++s_scan_seq;
    if ((s_scan_seq % 10) == 0) {
        float min_r = laser_msg_.range_max;
        float max_r = 0.0f;
        for (int b = 0; b < LIDAR_BUCKET_COUNT; ++b) {
            if (std::isfinite(ranges_[b])) {
                if (ranges_[b] < min_r) min_r = ranges_[b];
                if (ranges_[b] > max_r) max_r = ranges_[b];
            }
        }
        ESP_LOGI(TAG_LIDAR_PUB,
                 "scan #%lu pts=%u mapped=%u invalid=%u overflow=%u "
                 "filled=%d/%d r=[%.2f,%.2f]m dt=%.1fms ok=%lu fail=%lu",
                 (unsigned long)s_scan_seq,
                 (unsigned)scan.point_count,
                 (unsigned)points_mapped,
                 (unsigned)points_invalid,
                 (unsigned)points_overflow,
                 filled_buckets, LIDAR_BUCKET_COUNT,
                 min_r, max_r,
                 scan_time_s * 1000.0f,
                 (unsigned long)s_pub_ok_total,
                 (unsigned long)s_pub_fail_total);
    }

    // ── Publish ───────────────────────────────────────────────────────────
    const rcl_ret_t ret = rcl_publish(&laser_pub_, &laser_msg_, NULL);
    if (ret != RCL_RET_OK) {
        ++s_pub_fail_streak;
        ++s_pub_fail_total;
        rcl_reset_error();
        incrementPubFailCount();

        if (s_pub_fail_streak <= 5 || (s_pub_fail_streak % 10) == 0) {
            ESP_LOGE(TAG_LIDAR_PUB,
                     "laser_scan rcl_publish FAILED ret=%ld streak=%lu total_fail=%lu pts=%u",
                     (long)ret,
                     (unsigned long)s_pub_fail_streak,
                     (unsigned long)s_pub_fail_total,
                     (unsigned)scan.point_count);
        }
    } else {
        s_pub_fail_streak = 0;
        ++s_pub_ok_total;
        ESP_LOGD(TAG_LIDAR_PUB, "laser_scan published OK seq=%lu", (unsigned long)s_scan_seq);
    }
}
