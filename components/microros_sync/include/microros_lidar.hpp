#pragma once
#include "microros_common.hpp"
#include <lidar_scan.hpp>
#include <sensor_msgs/msg/laser_scan.h>

// Number of angular output buckets.
// 360 → 1°/bucket; serialised ≈ 2940 bytes (fits in RMW_UXRCE_MAX_MESSAGE_SIZE=4096).
static constexpr int LIDAR_BUCKET_COUNT = 360;

// ─── Diagnostic result structs ───────────────────────────────────────────────

struct AccumStats {
    uint32_t points_total    = 0;  ///< Points processed
    uint32_t points_mapped   = 0;  ///< Points written to fine grid (closest-wins)
    uint32_t points_invalid  = 0;  ///< Points with angle outside [0, 360)
    uint32_t points_overflow = 0;  ///< Points with distance > 12 000 mm
    int      fine_filled     = 0;  ///< Fine-grid slots that received a valid reading
};

struct DownsampleStats {
    int   filled_buckets = 0;
    int   empty_buckets  = 0;
    float range_min      = 12.0f;
    float range_max      = 0.0f;
};

// ─── LidarComponent ─────────────────────────────────────────────────────────

class LidarComponent {
public:
    LidarComponent();

    bool init(rcl_node_t* node, rclc_executor_t* executor);
    bool fini(rcl_node_t* node);

    /**
     * @brief Publish a completed 360° scan.
     *
     * Gates on isTimeSynced(): if time is not yet synced the scan is
     * silently dropped and a periodic warning is logged.  All timestamps
     * use wall-clock epoch via ShelfbotTimestamp.
     */
    void publishLidarScan(const LidarScan& scan);

    // ── Isolated pipeline stages (public for testability) ─────────────────

    /**
     * @brief Map scan points into the internal 720-slot fine-resolution grid.
     *
     * angles_deg[] is assumed to be correctly populated by lidar_sensor's
     * interpolate_angle() (verified from source).  No re-interpolation is
     * performed here.  Closest-wins per fine slot.
     *
     * Caller must ensure the fine grid is reset (publishLidarScan() does this
     * automatically before calling accumulateScan()).
     */
    void accumulateScan(const LidarScan& scan, AccumStats& stats);

    /**
     * @brief Downsample the 720-slot fine grid into 360 output buckets.
     *
     * Each output bucket covers two adjacent fine slots; closest-wins across
     * the pair.  Results are written into the arrays backing laser_msg_.
     */
    void downsampleGrid(DownsampleStats& stats);

private:
    rcl_publisher_t             laser_pub_;
    sensor_msgs__msg__LaserScan laser_msg_;
    float                       ranges_[LIDAR_BUCKET_COUNT];
    float                       intensities_[LIDAR_BUCKET_COUNT];
};
