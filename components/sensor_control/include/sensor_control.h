#pragma once

// Centralized compile-time sensor presence
#define SHELFBOT_HAS_ULTRASONIC 1
#define SHELFBOT_HAS_TOF 1
#define SHELFBOT_HAS_LIDAR 1

// Centralized ToF/LiDAR driver selection (choose exactly one)
#define SHELFBOT_DRIVER_VL53L0X 0
#define SHELFBOT_DRIVER_VL53L1 0
#define SHELFBOT_DRIVER_VL53L1_MODBUS 0
#define SHELFBOT_DRIVER_LYDSTO 1
