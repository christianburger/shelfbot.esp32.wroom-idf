#pragma once
#include <idf_c_includes.hpp>

#define NUM_MOTORS 5
#define MOTOR_STOP_TO_SETUP_DELAY_MS 5000

// Lifecycle functions
void motor_control_setup();
void motor_control_init();
void motor_control_start();
void motor_control_stop();
void motor_control_update();

// Original motor control API
void motor_control_begin();
void motor_control_set_position(uint8_t motor, float position);
void motor_control_set_velocity(uint8_t motor, float velocity);
float motor_control_get_position(uint8_t motor);

// Additional functions required by HTTP server
float motor_control_get_velocity(uint8_t motor);
bool motor_control_is_motor_running(uint8_t motor);
void motor_control_apply(uint8_t motor, float position, float velocity);