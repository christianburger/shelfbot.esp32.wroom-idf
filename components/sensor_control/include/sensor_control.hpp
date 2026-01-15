#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define NUM_SENSORS 2

// Queue for sending sensor data to the ROS publisher
extern QueueHandle_t distance_data_queue;

// Queue for sending emergency stop signals to the main loop
extern QueueHandle_t motor_stop_queue;

// Initializes the sensor component and creates the queues
void sensor_control_init();

// Starts the background task that reads sensors
void sensor_control_start_task();
