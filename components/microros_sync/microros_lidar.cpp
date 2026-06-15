// microros_lidar.cpp
//
// CHANGES vs previous version
// ════════════════════════════
//
// 1. ROOT CAUSE CORRECTION (lidar_sensor.cpp, not here)
//    The angles_deg[] array in LidarScan is populated correctly by
//    lidar_sensor.cpp's interpolate_angle().  The "re-interpolation" logic in
//    the previous microros_lidar.cpp was solving a bug that does not exist
//    and has been removed.
//
//    The real root causes were in lidar_sensor.cpp:
//      a) is_wrap_around() 120° threshold too tight: CRC failures eating
//         >120° of packets around the 0°/360° boundary caused wrap detection
//         to miss, merging multiple revolutions.  Fixed: threshold → 300°.
//      b) No time-based fallback: added MAX_SCAN_DURATION_US (250ms) guard.
//
// 2. SIMPLIFIED accumulateScan()
//    Angles are already correct.  The method simply maps each point into the
//    nearest 0.5° fine-grid slot (closest-wins), with wraparound clamping.
//
// 3. ISOLATED METHODS (as requested)
//    accumulateScan() and downsampleGrid() are separate, testable methods.
//
// 4. TIME-SYNC GATE
//    publishLidarScan() returns immediately (with a periodic warning) if
//    isTimeSynced() is false.  Note: the lidar_sensor state machine already
//    gates behind time_sync via the shelfbot RUNNING prerequisite, so in
//    normal operation this guard fires only during the brief startup window.
//
// 5. EPOCH TIMESTAMPS
//    header.stamp is set from wall-clock epoch, derived from the scan's
//    monotonic timestamps via a single-snapshot clock offset to minimise
//    the conversion race window.  scan_time comes from the monotonic delta
//    (reliable), stamp from epoch (required by ROS 2).
//
// 6. DIAGNOSTICS
//    - First DIAG_FULL_SCANS scans: every input point is logged.
//    - Every scan: summary (point counts, fill stats, range, timing).
//    - Every DIAG_PERIOD scans: run-length encoded output bucket histogram.
//    - Warning when output is < 25% filled (indicates upstream data loss).

#include "microros_lidar.hpp"
#include "microros_common.hpp"
#include <shelfbot_timestamp.hpp>
#include <cmath>
#include <limits>
#include <cstdio>

static const char* TAG = "LidarPub";

// ─── Fine-grid constants ─────────────────────────────────────────────────────
// 720 slots at 0.5°/slot.  The LYDSTO interpolates 12 points across ~2.5°,
// giving ~0.23°/point.  At 0.5° resolution consecutive points land in
// different slots, so no slot is structurally empty after a full revolution.
// FINE_SLOTS must be an integer multiple of LIDAR_BUCKET_COUNT.
static constexpr int FINE_SLOTS = 720;
static_assert(FINE_SLOTS % LIDAR_BUCKET_COUNT == 0,
              "FINE_SLOTS must be a multiple of LIDAR_BUCKET_COUNT");
static constexpr int RATIO = FINE_SLOTS / LIDAR_BUCKET_COUNT;  // = 2

// ─── Diagnostic tuning ───────────────────────────────────────────────────────
static constexpr uint32_t DIAG_FULL_SCANS = 5;   // log every point for first N scans
static constexpr uint32_t DIAG_PERIOD     = 20;  // full histogram every N scans

// ─── Module-level storage ────────────────────────────────────────────────────
// Static to avoid stack pressure; safe because publishLidarScan() is called
// only from the executor thread (sequential, never re-entrant).
static float s_fine_ranges[FINE_SLOTS];
static float s_fine_intensities[FINE_SLOTS];

static uint32_t s_scan_seq        = 0;
static uint32_t s_pub_ok_total    = 0;
static uint32_t s_pub_fail_total  = 0;
static uint32_t s_pub_fail_streak = 0;

// ─── Fine-grid reset ─────────────────────────────────────────────────────────
static void resetFineGrid()
{
    const float inf = std::numeric_limits<float>::infinity();
    for (int i = 0; i < FINE_SLOTS; ++i) {
        s_fine_ranges[i]      = inf;
        s_fine_intensities[i] = 0.0f;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// accumulateScan
//
// Maps each point in scan into the 720-slot fine grid using closest-wins.
//
// angles_deg[] is populated correctly by lidar_sensor.cpp's interpolate_angle()
// which linearly interpolates each of the 12 per-packet points between
// packet.start_angle_deg and packet.end_angle_deg.  No correction needed here.
//
// Angle clamping: values outside [0, 360) are wrapped with fmodf.
// These should be rare (interpolate_angle already clamps) but we guard anyway.
//
// Diagnostic (first DIAG_FULL_SCANS scans): logs every input point.
// ─────────────────────────────────────────────────────────────────────────────
void LidarComponent::accumulateScan(const LidarScan& scan, AccumStats& stats)
{
    stats = {};
    const bool verbose = (s_scan_seq <= DIAG_FULL_SCANS);
    const float inf = std::numeric_limits<float>::infinity();

    for (uint16_t i = 0; i < scan.point_count; ++i) {
        stats.points_total++;

        // Clamp angle to [0, 360)
        float angle_deg = scan.angles_deg[i];
        if (!std::isfinite(angle_deg) || angle_deg < 0.0f || angle_deg >= 360.0f) {
            angle_deg = fmodf(angle_deg, 360.0f);
            if (angle_deg < 0.0f) angle_deg += 360.0f;
            // If still out of range after clamp (e.g. NaN), skip
            if (!std::isfinite(angle_deg) || angle_deg < 0.0f || angle_deg >= 360.0f) {
                stats.points_invalid++;
                if (verbose) {
                    ESP_LOGW(TAG, "[DIAG] pt[%u] invalid angle=%.3f — skipped",
                             i, scan.angles_deg[i]);
                }
                continue;
            }
        }

        // Map to nearest fine slot (round, not floor)
        const int fine = static_cast<int>(
            lroundf(angle_deg * (float)FINE_SLOTS / 360.0f)
        ) % FINE_SLOTS;

        const uint16_t d_mm = scan.distances_mm[i];
        float r_m;
        if (d_mm == 0 || d_mm > 12000) {
            r_m = inf;
            if (d_mm > 12000) stats.points_overflow++;
        } else {
            r_m = (float)d_mm / 1000.0f;
        }

        if (verbose) {
            ESP_LOGI(TAG, "[DIAG] pt[%3u] angle=%.2f° slot=%3d d=%u mm r=%.3f m conf=%u",
                     i, angle_deg, fine, d_mm, r_m,
                     (unsigned)scan.confidences[i]);
        }

        // Closest-wins into fine grid
        if (r_m < s_fine_ranges[fine]) {
            s_fine_ranges[fine]      = r_m;
            s_fine_intensities[fine] = (float)scan.confidences[i];
            stats.points_mapped++;
        }
    }

    // Count filled fine slots
    for (int i = 0; i < FINE_SLOTS; ++i) {
        if (std::isfinite(s_fine_ranges[i])) stats.fine_filled++;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// downsampleGrid
//
// Collapses 720 fine slots into 360 output buckets (2 fine slots per bucket).
// Closest-wins across each pair.  Results go into ranges_[] / intensities_[].
//
// Diagnostic: every DIAG_PERIOD scans, logs a run-length encoded histogram.
// ─────────────────────────────────────────────────────────────────────────────
void LidarComponent::downsampleGrid(DownsampleStats& stats)
{
    stats = {};

    for (int b = 0; b < LIDAR_BUCKET_COUNT; ++b) {
        ranges_[b]      = std::numeric_limits<float>::infinity();
        intensities_[b] = 0.0f;
        for (int f = 0; f < RATIO; ++f) {
            const int fine = b * RATIO + f;
            if (s_fine_ranges[fine] < ranges_[b]) {
                ranges_[b]      = s_fine_ranges[fine];
                intensities_[b] = s_fine_intensities[fine];
            }
        }
        if (std::isfinite(ranges_[b])) {
            stats.filled_buckets++;
            if (ranges_[b] < stats.range_min) stats.range_min = ranges_[b];
            if (ranges_[b] > stats.range_max) stats.range_max = ranges_[b];
        }
    }
    stats.empty_buckets = LIDAR_BUCKET_COUNT - stats.filled_buckets;

    // Run-length encoded histogram every DIAG_PERIOD scans
    if (s_scan_seq > 0 && (s_scan_seq % DIAG_PERIOD) == 0) {
        char  buf[300];
        int   bpos = 0;
        int   b    = 0;
        while (b < LIDAR_BUCKET_COUNT && bpos < (int)sizeof(buf) - 32) {
            const bool is_inf = !std::isfinite(ranges_[b]);
            const float val   = ranges_[b];
            int run = 1;
            while (b + run < LIDAR_BUCKET_COUNT) {
                const bool next_inf = !std::isfinite(ranges_[b + run]);
                if (next_inf != is_inf) break;
                if (!is_inf && fabsf(ranges_[b + run] - val) >= 0.05f) break;
                run++;
            }
            if (is_inf) {
                bpos += snprintf(buf + bpos, sizeof(buf) - bpos, "inf×%d ", run);
            } else {
                bpos += snprintf(buf + bpos, sizeof(buf) - bpos, "%.2f×%d ", val, run);
            }
            b += run;
        }
        ESP_LOGI(TAG, "[DIAG] scan#%lu buckets: %s",
                 (unsigned long)s_scan_seq, buf);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// LidarComponent constructor
// ─────────────────────────────────────────────────────────────────────────────
LidarComponent::LidarComponent()
{
    sensor_msgs__msg__LaserScan__init(&laser_msg_);

    laser_msg_.ranges.data          = ranges_;
    laser_msg_.ranges.capacity      = LIDAR_BUCKET_COUNT;
    laser_msg_.ranges.size          = LIDAR_BUCKET_COUNT;
    laser_msg_.intensities.data     = intensities_;
    laser_msg_.intensities.capacity = LIDAR_BUCKET_COUNT;
    laser_msg_.intensities.size     = LIDAR_BUCKET_COUNT;

    laser_msg_.range_min = 0.02f;
    laser_msg_.range_max = 12.0f;

    const float inc = (float)(2.0 * M_PI) / (float)LIDAR_BUCKET_COUNT;
    laser_msg_.angle_min       = 0.0f;
    laser_msg_.angle_increment = inc;
    laser_msg_.angle_max       = (float)(LIDAR_BUCKET_COUNT - 1) * inc;
    laser_msg_.time_increment  = 0.0f;
    laser_msg_.scan_time       = 0.0f;

    static char frame_id_buf[] = "laser_link";
    laser_msg_.header.frame_id = {
        frame_id_buf,
        sizeof(frame_id_buf) - 1,
        sizeof(frame_id_buf)
    };

    laser_pub_ = rcl_get_zero_initialized_publisher();

    const float inf = std::numeric_limits<float>::infinity();
    for (int i = 0; i < LIDAR_BUCKET_COUNT; ++i) {
        ranges_[i]      = inf;
        intensities_[i] = 0.0f;
    }
    resetFineGrid();
}

// ─────────────────────────────────────────────────────────────────────────────
// init / fini
// ─────────────────────────────────────────────────────────────────────────────
bool LidarComponent::init(rcl_node_t* node, rclc_executor_t* executor)
{
    (void)executor;

    laser_pub_        = rcl_get_zero_initialized_publisher();
    s_pub_fail_streak = 0;
    s_pub_ok_total    = 0;
    s_pub_fail_total  = 0;
    s_scan_seq        = 0;
    resetFineGrid();

    const rcl_ret_t r = rclc_publisher_init_default(
        &laser_pub_, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "shelfbot_firmware/laser_scan");

    if (r != RCL_RET_OK) {
        ESP_LOGE(TAG, "laser_pub init FAILED: %ld", (long)r);
        return false;
    }

    ESP_LOGI(TAG,
        "laser_pub init OK  topic=shelfbot_firmware/laser_scan "
        "out_buckets=%d fine_slots=%d ratio=%d",
        LIDAR_BUCKET_COUNT, FINE_SLOTS, RATIO);
    return true;
}

bool LidarComponent::fini(rcl_node_t* node)
{
    const rcl_ret_t r = rcl_publisher_fini(&laser_pub_, node);
    if (r != RCL_RET_OK) {
        ESP_LOGE(TAG, "laser_pub fini FAILED: %ld", (long)r);
        return false;
    }
    laser_pub_ = rcl_get_zero_initialized_publisher();
    ESP_LOGI(TAG, "laser_pub fini OK  session: ok=%lu fail=%lu",
             (unsigned long)s_pub_ok_total, (unsigned long)s_pub_fail_total);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// publishLidarScan
// ─────────────────────────────────────────────────────────────────────────────
void LidarComponent::publishLidarScan(const LidarScan& scan)
{
    // ── Gate 1: data sanity ───────────────────────────────────────────────
    if (!scan.complete || scan.point_count == 0) {
        ESP_LOGD(TAG, "publishLidarScan: skip (complete=%d pts=%u)",
                 (int)scan.complete, (unsigned)scan.point_count);
        return;
    }

    // ── Gate 2: time must be synced ───────────────────────────────────────
    // The lidar_sensor state machine already gates RUNNING behind
    // shelfbot RUNNING which requires time_sync SYNCED, so this fires only
    // in the brief startup window.  Belt-and-suspenders guard.
    if (!isTimeSynced()) {
        static uint32_t s_nosync_count = 0;
        if ((++s_nosync_count % 25) == 0) {
            ESP_LOGW(TAG,
                "publishLidarScan: time not synced — suppressing "
                "(dropped %lu scans)", (unsigned long)s_nosync_count);
        }
        return;
    }

    ++s_scan_seq;

    // ── Timestamps ────────────────────────────────────────────────────────
    // scan.start/end_time_us are monotonic (esp_timer_get_time()).
    // ROS header.stamp must be wall-clock epoch.
    // Capture both clocks in one atomic-ish snapshot to minimise the race.
    const shelfbot::Timestamp now_snap  = shelfbot::ShelfbotTimestamp::now();
    const int64_t mono_to_epoch_us      = now_snap.epoch_us - now_snap.monotonic_us;

    // scan_time from the reliable monotonic delta (not affected by NTP steps)
    float scan_time_s = 0.0f;
    if (scan.end_time_us > scan.start_time_us) {
        scan_time_s = (float)(scan.end_time_us - scan.start_time_us) * 1e-6f;
    }
    if (scan_time_s > 1.0f) {
        // Should not happen after the lidar_sensor time-based wrap fix, but log if it does.
        ESP_LOGW(TAG,
            "scan #%lu: scan_time=%.3fs is suspiciously large "
            "(expected ~0.17s at 6Hz). "
            "Check lidar_sensor wrap detection.",
            (unsigned long)s_scan_seq, scan_time_s);
    }

    laser_msg_.scan_time      = scan_time_s;
    laser_msg_.time_increment = (scan_time_s > 0.0f)
        ? scan_time_s / (float)LIDAR_BUCKET_COUNT
        : 0.0f;

    // header.stamp = wall-clock time of the END of the scan
    const int64_t stamp_epoch_us = scan.end_time_us + mono_to_epoch_us;
    shelfbot::ShelfbotTimestamp::toRosTime(
        stamp_epoch_us,
        laser_msg_.header.stamp.sec,
        laser_msg_.header.stamp.nanosec);

    // ── Stage 1: accumulate into fine grid ───────────────────────────────
    resetFineGrid();
    AccumStats accum;
    accumulateScan(scan, accum);

    // ── Stage 2: downsample → output buckets ─────────────────────────────
    DownsampleStats ds;
    downsampleGrid(ds);

    // ── Summary log ───────────────────────────────────────────────────────
    const bool do_log = (s_scan_seq <= DIAG_FULL_SCANS) ||
                        ((s_scan_seq % DIAG_PERIOD) == 0);
    if (do_log) {
        ESP_LOGI(TAG,
            "scan #%lu  pts=%u  mapped=%u  invalid=%u  overflow=%u  "
            "fine_filled=%d/%d  out_filled=%d/%d  empty=%d  "
            "r=[%.2f,%.2f]m  dt=%.1fms  ok=%lu fail=%lu",
            (unsigned long)s_scan_seq,
            (unsigned)scan.point_count,
            (unsigned)accum.points_mapped,
            (unsigned)accum.points_invalid,
            (unsigned)accum.points_overflow,
            accum.fine_filled, FINE_SLOTS,
            ds.filled_buckets, LIDAR_BUCKET_COUNT,
            ds.empty_buckets,
            ds.range_min, ds.range_max,
            scan_time_s * 1000.0f,
            (unsigned long)s_pub_ok_total,
            (unsigned long)s_pub_fail_total);

        if (ds.filled_buckets < LIDAR_BUCKET_COUNT / 4) {
            ESP_LOGW(TAG,
                "scan #%lu: only %d/%d output buckets filled (<25%%). "
                "Likely UART packet loss — check CRC fail rate in LidarSensor.",
                (unsigned long)s_scan_seq,
                ds.filled_buckets, LIDAR_BUCKET_COUNT);
        }
    }

    // ── Publish ───────────────────────────────────────────────────────────
    const rcl_ret_t ret = rcl_publish(&laser_pub_, &laser_msg_, nullptr);
    if (ret != RCL_RET_OK) {
        ++s_pub_fail_streak;
        ++s_pub_fail_total;
        rcl_reset_error();
        incrementPubFailCount();
        if (s_pub_fail_streak <= 5 || (s_pub_fail_streak % 10) == 0) {
            ESP_LOGE(TAG,
                "laser_scan rcl_publish FAILED ret=%ld streak=%lu total=%lu",
                (long)ret,
                (unsigned long)s_pub_fail_streak,
                (unsigned long)s_pub_fail_total);
        }
    } else {
        s_pub_fail_streak = 0;
        ++s_pub_ok_total;
        ESP_LOGD(TAG, "laser_scan published OK seq=%lu", (unsigned long)s_scan_seq);
    }
}
