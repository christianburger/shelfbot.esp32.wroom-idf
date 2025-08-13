#pragma once

#include <stdint.h>

// The number of ultrasonic sensors.
#define NUM_ULTRASONIC_SENSORS 8

/**
 * @brief Initializes the GPIO pins and RMT channels for the ultrasonic sensors.
 *
 * This function sets up the multiplexer select pins, the trigger pin, and the
 * echo pin. It also configures the RMT peripheral for sending the trigger
 * pulse and measuring the echo pulse duration without blocking.
 */
void ultrasonic_sensors_init();

/**
 * @brief Reads all 8 ultrasonic sensors in sequence.
 *
 * @param distances A pointer to a float array of size NUM_ULTRASONIC_SENSORS.
 *                  The measured distances in centimeters for each sensor will be
 *                  stored in this array. The caller is responsible for allocating
 *                  this memory.
 */
void ultrasonic_sensors_read_all(float* distances);
