#include "shelfbot_timestamp.hpp"
#include <sys/time.h>
#include <ctime>

namespace shelfbot {

static constexpr time_t kReasonableEpoch = 1700000000LL;

int64_t ShelfbotTimestamp::monotonicMicros() {
    return esp_timer_get_time();
}

int64_t ShelfbotTimestamp::epochMicros() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return static_cast<int64_t>(tv.tv_sec) * 1000000LL
         + static_cast<int64_t>(tv.tv_usec);
}

Timestamp ShelfbotTimestamp::now() {
    Timestamp ts;
    ts.monotonic_us = monotonicMicros();
    ts.epoch_us     = epochMicros();
    return ts;
}

bool ShelfbotTimestamp::isEpochValid() {
    time_t now_sec;
    time(&now_sec);
    return now_sec > kReasonableEpoch;
}

void ShelfbotTimestamp::toRosTime(int64_t epoch_us, int32_t& sec, uint32_t& nanosec) {
    sec     = static_cast<int32_t>(epoch_us / 1000000LL);
    nanosec = static_cast<uint32_t>((epoch_us % 1000000LL) * 1000LL);
}

FrameTimestamp ShelfbotTimestamp::capture(uint32_t sequence) {
    FrameTimestamp ts;
    ts.sequence            = sequence;
    ts.capture_monotonic_us = monotonicMicros();
    ts.capture_epoch_us    = epochMicros();
    return ts;
}

}