#include <led_control.hpp>
#include <state_machine.hpp>
#include <state_machine_lifecycle.hpp>

static const char* TAG = "LED_Control";
static TaskHandle_t led_task_handle = nullptr;
static bool led_state = false;

static void led_lifecycle_task(void* /*arg*/) {
    static const uint32_t RETRY_MS = 1000;

    while (!StateMachine::isInState("led_control", stateToString(LedControlState::INIT))) {
        StateMachine::advance("led_control");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }
    ESP_LOGI(TAG, "LED init done");

    while (!StateMachine::isInState("led_control", stateToString(LedControlState::RUNNING))) {
        StateMachine::advance("led_control");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }
    ESP_LOGI(TAG, "LED running");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(nullptr);
}

void led_control_setup() {
    ESP_LOGI(TAG, "LED setup: spawning lifecycle task");
    xTaskCreate(led_lifecycle_task, "led_lifecycle", 2048, nullptr, 3, &led_task_handle);
}

void led_control_init()   {}
void led_control_start()  {}
void led_control_stop()   {}
void led_control_update() {}

bool led_control_get() {
    return led_state;
}

void led_control_set(bool state) {
    led_state = state;
    ESP_LOGI(TAG, "LED set to %d", state);
}