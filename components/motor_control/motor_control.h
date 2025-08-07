#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include <vector>

#define NUM_MOTORS 6
#define MICROSTEPPING 16
#define STEPS_PER_REVOLUTION 100
#define GEAR_RATIO 1

void motor_control_begin();
long motor_control_get_motor_speed(uint8_t index);
void motor_control_set_motor_speed(uint8_t index, long speed);
void motor_control_get_all_motor_speeds(long* speeds, size_t* num_speeds);
void motor_control_set_all_motor_speeds(long speed);
void motor_control_move_all_motors(long position, long speed, bool non_blocking);
void motor_control_non_blocking_move_all_motors(long position);
bool motor_control_are_all_motors_stopped();
void motor_control_set_motor_position(uint8_t index, long position);
long motor_control_get_motor_position(uint8_t index);
long motor_control_get_motor_velocity(uint8_t index);
void motor_control_stop_motor(uint8_t index);
void motor_control_stop_all_motors();
bool motor_control_is_motor_running(uint8_t index);
double motor_control_get_motor_position_double(uint8_t index);
double motor_control_get_motor_velocity_double(uint8_t index);
bool motor_control_move_all_motors_vector(const double* positions, size_t num_positions, long speed, bool non_blocking);
void motor_control_set_motor_position_double(uint8_t index, double position);

#endif // MOTOR_CONTROL_H
