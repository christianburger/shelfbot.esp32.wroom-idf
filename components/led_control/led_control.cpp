#include <led_control.hpp>
#include <state_machine.hpp>
#include <state_machine_lifecycle.hpp>

#define BLINK_GPIO GPIO_NUM_2

static const char* TAG    = "LED_Control";
static bool        led_state = false;

// ---------------------------------------------------------------------------
// Hardware helpers
// ---------------------------------------------------------------------------
static void hw_configure_gpio() {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask  = (1ULL << BLINK_GPIO);
    io_conf.mode          = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);
    gpio_set_level(BLINK_GPIO, 0);
}

// ---------------------------------------------------------------------------
// Self-driving lifecycle task
// ---------------------------------------------------------------------------
static void led_lifecycle_task(void* /*arg*/) {
    static const uint32_t RETRY_MS = 1000;

    // setup -> init
    while (!StateMachine::canTransition("led_control",
                                        stateToString(LedControlState::INIT))) {
        ESP_LOGD(TAG, "waiting for prereqs: setup->init");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }
    hw_configure_gpio();
    StateMachine::changeState("led_control", stateToString(LedControlState::INIT));
    ESP_LOGI(TAG, "LED init done");

    // init -> running
    while (!StateMachine::canTransition("led_control",
                                        stateToString(LedControlState::RUNNING))) {
        ESP_LOGD(TAG, "waiting for prereqs: init->running");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }
    StateMachine::changeState("led_control", stateToString(LedControlState::RUNNING));
    ESP_LOGI(TAG, "LED running");

    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// Public lifecycle API
// ---------------------------------------------------------------------------
void led_control_setup() {
    ESP_LOGI(TAG, "LED setup: spawning lifecycle task");
    xTaskCreate(led_lifecycle_task, "led_lifecycle", 2048, nullptr, 3, nullptr);
}

void led_control_init()   {}  // driven by lifecycle task
void led_control_start()  {}  // driven by lifecycle task
void led_control_stop() {
    led_control_set(false);
    StateMachine::changeState("led_control",
                               stateToString(LedControlState::STOPPED),
                               /*force=*/true);
}
void led_control_update() {}

// ---------------------------------------------------------------------------
// Runtime API
// ---------------------------------------------------------------------------
void led_control_set(bool state) {
    led_state = state;
    gpio_set_level(BLINK_GPIO, led_state ? 1 : 0);
}

bool led_control_get_state() { return led_state; }