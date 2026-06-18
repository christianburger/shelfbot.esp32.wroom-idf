#pragma once
#include <idf_c_includes.hpp>

#define LED_STOP_TO_SETUP_DELAY_MS 5000

// ---------------------------------------------------------------------------
// Task function — created by shelfbot.cpp.
// Stack budget: 2048 words (8 KB).
// ---------------------------------------------------------------------------
void led_lifecycle_task_fn(void* arg);

// Lifecycle functions
void led_control_setup();   // initialises state; does NOT spawn task
void led_control_init();
void led_control_start();
void led_control_stop();
void led_control_update();

// Legacy interface
void led_control_set(bool state);
bool led_control_get_state();
