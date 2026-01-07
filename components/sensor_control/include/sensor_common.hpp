#pragma once
#ifndef SHELFBOT_SENSOR_COMMON_H
#define SHELFBOT_SENSOR_COMMON_H

#include <cstdint>
#include <vector>  // If needed for arrays, but keep minimal

namespace SensorCommon {

    // Unified reading structure (distance in cm for consistency; convert ToF mm to cm if desired)
    struct Reading {
        float distance_cm;    // Distance in centimeters (unified unit)
        uint8_t status;       // 0 = OK, >0 = error code (e.g., timeout, invalid)
        int64_t timestamp_us; // Microseconds since boot
        bool valid;           // Quick validity flag
    };

    // Constants shared across drivers (e.g., min/max values)
    constexpr float MIN_DISTANCE_CM = 2.0f;   // Typical min for both sensor types
    constexpr float MAX_DISTANCE_CM = 400.0f; // Adjust based on sensor specs
    constexpr uint32_t DEFAULT_TIMEOUT_MS = 100;

}  // namespace SensorCommon
#endif //SHELFBOT_SENSOR_COMMON_H