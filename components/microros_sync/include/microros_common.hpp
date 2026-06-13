// microros_common.hpp
#pragma once
#include <idf_c_includes.hpp>
#include <rcl/rcl.h>
#include <rclc/rclc.h>

struct MicrorosCore;
extern MicrorosCore* g_microros_core;

// Lock helpers (implemented in microros_sync.cpp)
bool lockCore();
void unlockCore();

// State queries (implemented in microros_common.cpp)
bool isMicrorosConnected();
bool isTimeSynced();

// Timestamp filling (implemented in microros_common.cpp)
void fillRosStamp(int32_t& sec, uint32_t& nanosec);

// Publish failure counter (implemented in microros_common.cpp)
void incrementPubFailCount();
uint32_t getPubFailCount();
void resetPubFailCount();

// Helper: replace PUB_OR_FAIL macro
bool publish_or_fail(rcl_publisher_t* publisher, const void* msg, const char* label);