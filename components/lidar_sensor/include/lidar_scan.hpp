#pragma once
#include <cstdint>
#include <cstddef>

/**
 * @brief One complete 360° scan from a rotating LiDAR.
 *
 * Points are stored in the order they were received from the sensor.
 * angles_deg[] runs monotonically through the arc that was captured;
 * after a full revolution they span [0, 360).
 *
 * MAX_POINTS is sized for the LYDSTO LDS02RR at ~12 points per packet
 * and ~15 packets per revolution (practical maximum ~180 points), but
 * 2000 is kept as a safe ceiling for any LD06/LD19-family sensor at its
 * finest angular resolution (0.18° → 2000 points/rev).
 *
 * Usage:
 *   LidarScan scan;
 *   scan.clear();               // reset before collecting
 *   // ... accumulate via LidarAccumulator ...
 *   if (scan.complete) { ... }  // use it
 */
struct LidarScan {
    static constexpr uint16_t MAX_POINTS = 2000;

    // ── Per-point arrays (parallel, indexed 0..point_count-1) ────────────
    uint16_t distances_mm[MAX_POINTS];   // range in millimetres
    uint8_t  confidences[MAX_POINTS];    // 0..255, higher = more reliable
    float    angles_deg[MAX_POINTS];     // angle of the point  [0.0, 360.0)

    uint16_t point_count;                // number of valid points in this scan

    // ── Timing ────────────────────────────────────────────────────────────
    int64_t  start_time_us;              // esp_timer_get_time() at first point
    int64_t  end_time_us;                // esp_timer_get_time() at last point

    // ── Status ────────────────────────────────────────────────────────────
    bool     complete;                   // true when a full 360° has been captured

    // ── Helpers ───────────────────────────────────────────────────────────
    void clear() {
        point_count    = 0;
        complete       = false;
        start_time_us  = 0;
        end_time_us    = 0;
    }

    float distance_cm(uint16_t idx) const {
        return (idx < point_count) ? distances_mm[idx] / 10.0f : 0.0f;
    }
};
