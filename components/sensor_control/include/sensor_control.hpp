#pragma once

#ifndef SHELFBOT_SENSOR_CONTROL_H
#define SHELFBOT_SENSOR_CONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sensor_common.hpp"
#include <cstdint>

#define NUM_ULTRASONIC_SENSORS 2
#define NUM_TOF_SENSORS 1
#define NUM_SENSORS (NUM_ULTRASONIC_SENSORS + NUM_TOF_SENSORS)

// Unified sensor data structure for the queue
struct SensorDataPacket {
  float ultrasonic_distances_cm[NUM_ULTRASONIC_SENSORS];
  float tof_distances_cm[NUM_TOF_SENSORS];
  int64_t timestamp_us;
  bool ultrasonic_valid[NUM_ULTRASONIC_SENSORS];
  bool tof_valid[NUM_TOF_SENSORS];
};

// Queue for sending unified sensor data to the ROS publisher
extern QueueHandle_t unified_sensor_data_queue;

// Queue for sending emergency stop signals
extern QueueHandle_t motor_stop_queue;

// Queue for legacy ultrasonic data
extern QueueHandle_t distance_data_queue;

// Initializes the sensor control component and creates queues
void sensor_control_init();

// Starts the background task that manages all sensors
void sensor_control_start_task();

// Gets the latest sensor data (non-blocking)
bool sensor_control_get_latest_data(SensorDataPacket* packet);

#endif // SHELFBOT_SENSOR_CONTROL_H