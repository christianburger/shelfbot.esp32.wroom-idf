#pragma once

#include <stdbool.h>

/**
 * @brief Initializes the GPIO for the built-in LED.
 */
void led_control_init(void);

/**
 * @brief Sets the state of the built-in LED.
 *
 * @param state true to turn the LED on, false to turn it off.
 */
void led_control_set(bool state);

/**
 * @brief Gets the current state of the built-in LED.
 *
 * @return true if the LED is on, false if it is off.
 */
bool led_control_get_state(void);
