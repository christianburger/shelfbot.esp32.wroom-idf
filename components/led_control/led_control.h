#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif
