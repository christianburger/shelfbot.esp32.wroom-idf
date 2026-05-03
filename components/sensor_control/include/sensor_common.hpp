// [file name]: sensor_common.hpp
#pragma once
#ifndef SHELFBOT_SENSOR_COMMON_H
#define SHELFBOT_SENSOR_COMMON_H
#include <idf_c_includes.hpp>

namespace SensorCommon {

// Compile-time feature toggles (override with -D at build time)
#ifndef SHELFBOT_ENABLE_ULTRASONIC
#define SHELFBOT_ENABLE_ULTRASONIC 1
#endif
#ifndef SHELFBOT_ENABLE_TOF
#define SHELFBOT_ENABLE_TOF 1
#endif
#ifndef SHELFBOT_ENABLE_LIDAR
#define SHELFBOT_ENABLE_LIDAR 1
#endif

#ifndef SHELFBOT_TOF_DRIVER_VL53L0X
#define SHELFBOT_TOF_DRIVER_VL53L0X 1
#endif
#ifndef SHELFBOT_TOF_DRIVER_VL53L1
#define SHELFBOT_TOF_DRIVER_VL53L1 0
#endif
#ifndef SHELFBOT_TOF_DRIVER_VL53L1_MODBUS
#define SHELFBOT_TOF_DRIVER_VL53L1_MODBUS 0
#endif
#ifndef SHELFBOT_TOF_DRIVER_LYDSTO
#define SHELFBOT_TOF_DRIVER_LYDSTO 0
#endif

  constexpr int NUM_ULTRASONIC_SENSORS = 4;
  constexpr int NUM_TOF_SENSORS = 1;
  constexpr int NUM_SENSORS = NUM_ULTRASONIC_SENSORS + NUM_TOF_SENSORS;

  // Ultrasonic sensor reading structure
  struct Reading {
    float distance_cm;    // Distance in centimeters
    uint8_t status;       // 0 = OK, >0 = error code
    int64_t timestamp_us; // Microseconds since boot
    bool valid;           // Quick validity flag

    Reading() : distance_cm(0.0f), status(0), timestamp_us(0), valid(false) {}
  };

  // ToF sensor data structure
  struct TofMeasurement {
    uint16_t distance_mm;
    bool valid;
    uint8_t status;       // 0 = OK, >0 = error code
    int64_t timestamp_us;
    bool timeout_occurred;

    TofMeasurement() : distance_mm(0), valid(false), status(0),
                       timestamp_us(0), timeout_occurred(false) {}

    // Helper to get distance in cm
    float distance_cm() const { return distance_mm / 10.0f; }
  };

  struct LidarMeasurement {
    uint16_t distance_mm;
    bool valid;
    uint8_t status;
    int64_t timestamp_us;
    bool timeout_occurred;

    LidarMeasurement() : distance_mm(0), valid(false), status(0),
                         timestamp_us(0), timeout_occurred(false) {}

    float distance_cm() const { return distance_mm / 10.0f; }
  };

  // Sensor data packet for sharing between components
  struct SensorDataPacket {
    // Ultrasonic sensors
    Reading ultrasonic_readings[NUM_ULTRASONIC_SENSORS];

    // ToF sensors
    TofMeasurement tof_measurements[NUM_TOF_SENSORS];

#if SHELFBOT_ENABLE_LIDAR
    LidarMeasurement lidar_measurement;
#endif

    int64_t timestamp_us;

    // Constructor to initialize arrays
    SensorDataPacket() : timestamp_us(0) {
      for (int i = 0; i < NUM_ULTRASONIC_SENSORS; i++) {
        ultrasonic_readings[i] = Reading();
      }
      for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        tof_measurements[i] = TofMeasurement();
      }
#if SHELFBOT_ENABLE_LIDAR
      lidar_measurement = LidarMeasurement();
#endif
    }
  };

  // Constants shared across drivers
  constexpr float MIN_DISTANCE_CM = 2.0f;
  constexpr float MAX_DISTANCE_CM = 400.0f;
  constexpr uint32_t DEFAULT_TIMEOUT_MS = 100;

  // Special value indicating "no target" from VL53L0X/VL53L1X
  constexpr float NO_TARGET_DISTANCE_CM = 819.0f;  // 8190mm converted to cm
  constexpr uint16_t NO_TARGET_DISTANCE_MM = 8190;  // VL53L0X "no target" value
}

#endif // SHELFBOT_SENSOR_COMMON_H