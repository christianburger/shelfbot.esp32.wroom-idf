#pragma once

#include "idf_c_includes.hpp"

namespace shelfbot {

// ---------------------------------------------------------------------------
// Raw timestamps captured at a single instant
// ---------------------------------------------------------------------------
struct Timestamp {
    int64_t monotonic_us;   ///< esp_timer_get_time() — always valid, starts at boot
    int64_t epoch_us;       ///< gettimeofday() — valid only after NTP/agent sync
};

// ---------------------------------------------------------------------------
// Per-frame timestamp bundle attached to sensor messages
// ---------------------------------------------------------------------------
struct FrameTimestamp {
    uint32_t sequence;              ///< Rolling frame counter
    int64_t  capture_monotonic_us;  ///< Monotonic time of capture
    int64_t  capture_epoch_us;      ///< Wall-clock time of capture (may be invalid)
};

// ---------------------------------------------------------------------------
// ShelfbotTimestamp — static utility class
// ---------------------------------------------------------------------------
class ShelfbotTimestamp {
public:
    ShelfbotTimestamp() = delete;

    /// Monotonic microseconds since boot (always valid).
    static int64_t monotonicMicros();

    /// Wall-clock microseconds since Unix epoch (valid only after clock sync).
    static int64_t epochMicros();

    /// Snapshot of both clocks at the same instant.
    static Timestamp now();

    /// Returns true when the wall clock has been set to a sane epoch (> 2023-01-01).
    static bool isEpochValid();

    /**
     * @brief Convert an epoch-microsecond value into ROS 2 builtin_interfaces/Time fields.
     * @param epoch_us   Input in microseconds since Unix epoch.
     * @param sec        Output: seconds component.
     * @param nanosec    Output: nanoseconds remainder.
     */
    static void toRosTime(int64_t epoch_us, int32_t& sec, uint32_t& nanosec);

    /// Convenience: capture a FrameTimestamp for the current moment.
    static FrameTimestamp capture(uint32_t sequence);
};

} // namespace shelfbot
