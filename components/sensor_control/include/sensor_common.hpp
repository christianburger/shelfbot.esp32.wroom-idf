#pragma once
#ifndef SHELFBOT_SENSOR_COMMON_H
#define SHELFBOT_SENSOR_COMMON_H
#include <idf_c_includes.hpp>

namespace SensorCommon {

  constexpr int NUM_ULTRASONIC_SENSORS = 4;
  constexpr int NUM_TOF_SENSORS = 1;
  constexpr int NUM_SENSORS = 5;

  // Unified reading structure (distance in cm for consistency)
  struct Reading {
    float distance_cm;    // Distance in centimeters (unified unit)
    uint8_t status;       // 0 = OK, >0 = error code (e.g., timeout, invalid)
    int64_t timestamp_us; // Microseconds since boot
    bool valid;           // Quick validity flag
  };

  // Sensor data packet for ROS publishing
  struct SensorDataPacket {
    float ultrasonic_distances_cm[NUM_ULTRASONIC_SENSORS];
    float tof_distances_cm[NUM_TOF_SENSORS];
    bool tof_valid[NUM_TOF_SENSORS];
    uint8_t tof_status[NUM_TOF_SENSORS];
    int64_t timestamp_us;
  };

  // Constants shared across drivers
  constexpr float MIN_DISTANCE_CM = 2.0f;
  constexpr float MAX_DISTANCE_CM = 400.0f;
  constexpr uint32_t DEFAULT_TIMEOUT_MS = 100;

  // Special value indicating "no target" from VL53L0X
  constexpr float NO_TARGET_DISTANCE_CM = 819.0f;  // 8190mm converted to cm
  constexpr uint16_t NO_TARGET_DISTANCE_MM = 8190;  // VL53L0X "no target" value
}

// C-style interface for sensor control
extern "C" {
  bool sensor_control_get_latest_data(SensorCommon::SensorDataPacket* data);
  void sensor_control_init(void);
  void sensor_control_start_task(void);
}

#endif // SHELFBOT_SENSOR_COMMON_H