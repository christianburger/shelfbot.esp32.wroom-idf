#include <led_control.hpp>
#include <state_machine.hpp>
#include <state_machine_lifecycle.hpp>

static const char* TAG = "LED_Control";
static bool led_state = false;

// ---------------------------------------------------------------------------
// Lifecycle task — created by shelfbot.cpp, NOT here.
// Exposed as led_lifecycle_task_fn() so shelfbot can size the stack correctly.
//
// Stack budget: 2048 words.
//   StateMachine::isInState / advance each acquire std::mutex + unordered_map
//   lookup (~1 KB frame on Xtensa).  No string construction or heap allocs
//   beyond that.  2048 words = 8 KB, well above the ~2 KB worst-case.
// ---------------------------------------------------------------------------
void led_lifecycle_task_fn(void* /*arg*/) {
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

// ---------------------------------------------------------------------------
// Public API — shelfbot.cpp calls led_control_setup() then creates the task.
// ---------------------------------------------------------------------------
void led_control_setup() {
    ESP_LOGI(TAG, "LED setup — task will be created by shelfbot");
    // Do NOT spawn a task here.  shelfbot.cpp creates led_lifecycle_task_fn.
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
