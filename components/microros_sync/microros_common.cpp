// microros_common.cpp
#include "microros_common.hpp"
#include <state_machine.hpp>
#include <state_machine_lifecycle.hpp>
#include <shelfbot_timestamp.hpp>

static uint32_t g_pub_fail_count = 0;

bool isMicrorosConnected() {
    return StateMachine::isInState("microros_sync", stateToString(MicrorosState::CONNECTED));
}

bool isTimeSynced() {
    return StateMachine::isAtLeast("time_sync", stateToString(TimeSyncState::SYNCED));
}

void fillRosStamp(int32_t& sec, uint32_t& nanosec) {
    if (isTimeSynced()) {
        shelfbot::ShelfbotTimestamp::toRosTime(
            shelfbot::ShelfbotTimestamp::epochMicros(), sec, nanosec);
    } else {
        int64_t mono = shelfbot::ShelfbotTimestamp::monotonicMicros();
        sec = static_cast<int32_t>(mono / 1000000LL);
        nanosec = static_cast<uint32_t>((mono % 1000000LL) * 1000LL);
    }
}

void incrementPubFailCount() { g_pub_fail_count++; }
uint32_t getPubFailCount() { return g_pub_fail_count; }
void resetPubFailCount() { g_pub_fail_count = 0; }

bool publish_or_fail(rcl_publisher_t* publisher, const void* msg, const char* label) {
    rcl_ret_t ret = rcl_publish(publisher, msg, NULL);
    if (ret != RCL_RET_OK) {
        ESP_LOGE("Microros", "%s pub failed: %ld", label, (long)ret);
        rcl_reset_error();
        incrementPubFailCount();
        return false;
    }
    return true;
}