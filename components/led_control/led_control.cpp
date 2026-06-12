#include <led_control.hpp>
#include <state_machine.hpp>
#include <state_machine_lifecycle.hpp>

#define BLINK_GPIO GPIO_NUM_2

static const char* TAG = "LED_Control";
static bool led_state = false;

void led_control_setup() {
    ESP_LOGI(TAG, "LED setup");
    StateMachine::changeState("led_control", stateToString(LedControlState::SETUP));
}

void led_control_init() {
    ESP_LOGI(TAG, "LED init");
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << BLINK_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);
    led_control_set(false);
    StateMachine::changeState("led_control", stateToString(LedControlState::INIT));
}

void led_control_start() {
    ESP_LOGI(TAG, "LED start");
    StateMachine::changeState("led_control", stateToString(LedControlState::RUNNING));
}

void led_control_stop() {
    StateMachine::changeState("led_control", stateToString(LedControlState::STOPPED));
}

void led_control_update() {
    // Called by orchestrator if needed
}

void led_control_set(bool state) {
    led_state = state;
    gpio_set_level(BLINK_GPIO, led_state ? 1 : 0);
}

bool led_control_get_state() {
    return led_state;
}