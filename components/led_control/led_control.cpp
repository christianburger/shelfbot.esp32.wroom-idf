#include "led_control.h"
#include "driver/gpio.h"

// Most ESP32-WROOM-32 boards have the built-in LED on GPIO 2.
#define BLINK_GPIO GPIO_NUM_2

// Variable to hold the current state of the LED
static bool led_state = false;

void led_control_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << BLINK_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);
    led_control_set(led_state); // Ensure LED is off on init
}

void led_control_set(bool state)
{
    led_state = state;
    gpio_set_level(BLINK_GPIO, led_state ? 1 : 0);
}

bool led_control_get_state(void)
{
    return led_state;
}
