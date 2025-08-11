#include "led_control.h"
#include "driver/gpio.h"

// Most ESP32-WROOM-32 boards have the built-in LED on GPIO 2.
#define BLINK_GPIO GPIO_NUM_2

void led_control_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << BLINK_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);
}

void led_control_set(bool state)
{
    gpio_set_level(BLINK_GPIO, state ? 1 : 0);
}
