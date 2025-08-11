#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include <vector>

// --- Hardware Configuration ---
#define NUM_MOTORS 6
#define STEPS_PER_REVOLUTION 200 // For standard 1.8-degree stepper motors
#define MICROSTEPPING 16
#define GEAR_RATIO 1

void motor_control_begin();

// --- ROS 2 Standard Interface (Primary Control) ---
// All position values are in RADIANS. All velocity values are in RADIANS/SEC.
void motor_control_set_position(uint8_t index, double position_rad);
double motor_control_get_position(uint8_t index);
double motor_control_get_velocity(uint8_t index);

// --- Utility Functions ---
void motor_control_set_speed_hz(uint8_t index, long speed_hz);
void motor_control_set_all_speeds_hz(long speed_hz);
void motor_control_stop_motor(uint8_t index);
void motor_control_stop_all_motors();
bool motor_control_is_motor_running(uint8_t index);

// --- DEPRECATED: Kept for REST API compatibility ---
void motor_control_set_motor_position_double(uint8_t index, double position);
double motor_control_get_motor_position_double(uint8_t index);
double motor_control_get_motor_velocity_double(uint8_t index);
bool motor_control_move_all_motors_vector(const double* positions, size_t num_positions, long speed, bool non_blocking);

#endif // MOTOR_CONTROL_H
