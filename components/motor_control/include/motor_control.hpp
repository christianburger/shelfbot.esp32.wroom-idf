#pragma once
#include <idf_c_includes.hpp>

#define NUM_MOTORS 5

// --- Hardware Configuration ---
#define STEPS_PER_REVOLUTION 200.0  // 1.8-degree stepper
#define MICROSTEPPING        8.0
#define GEAR_RATIO           1.0

// Radians → steps: (steps/rev × microsteps × gear) / (2π)
extern const double RADS_TO_STEPS;

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void motor_control_begin();

// ---------------------------------------------------------------------------
// Primary unified command
//
//  velocity_rad_s == 0              → stop
//  position_rad   == 0, vel ≠ 0    → run continuously in velocity direction
//  position_rad   ≠ 0, vel ≠ 0     → move to position at |velocity| speed
//  position_rad   ≠ 0, vel == 0    → move to position at default speed
// ---------------------------------------------------------------------------
void motor_control_apply(uint8_t index, float position_rad, float velocity_rad_s);

// ---------------------------------------------------------------------------
// ROS 2 standard interface — all units: radians, radians/sec
// ---------------------------------------------------------------------------
void  motor_control_set_velocity(uint8_t index, float velocity_rad_s);
void  motor_control_set_position(uint8_t index, float position_rad);
float motor_control_get_position(uint8_t index);
float motor_control_get_velocity(uint8_t index);  // signed — negative = reverse

// ---------------------------------------------------------------------------
// Utility
// ---------------------------------------------------------------------------
void motor_control_set_speed_hz(uint8_t index, long speed_hz);
void motor_control_set_all_speeds_hz(long speed_hz);
void motor_control_stop_motor(uint8_t index);
void motor_control_stop_all_motors();
bool motor_control_is_motor_running(uint8_t index);
