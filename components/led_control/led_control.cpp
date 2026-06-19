#include <led_control.hpp>
#include <state_machine.hpp>
#include <state_machine_lifecycle.hpp>

static const char* TAG = "LED_Control";
static bool led_state = false;

// ---------------------------------------------------------------------------
// Lifecycle task — created by shelfbot.cpp, NOT here.
//
// Stack budget: 2048 words (8 KB).
//   StateMachine::isInState / advance each acquire std::mutex + unordered_map
//   lookup (~1 KB frame on Xtensa).  No string construction or heap allocs
//   beyond that.  2048 words = 8 KB, well above the ~2 KB worst-case.
//
// Advance behaviour: if advance() returns false because shelfbot is not yet
// RUNNING (the gating prerequisite for led_control SETUP→INIT), the task
// simply delays 1 s and retries.  There is no error path — the conditions
// will eventually be met once WiFi, network, and time-sync converge.
// ---------------------------------------------------------------------------
void led_lifecycle_task_fn(void* /*arg*/) {
    static const uint32_t RETRY_MS = 1000;

    // SETUP → INIT
    // Prerequisite: shelfbot >= RUNNING.  Retries until satisfied.
    while (!StateMachine::isInState("led_control", stateToString(LedControlState::INIT))) {
        StateMachine::advance("led_control");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }
    ESP_LOGI(TAG, "LED init done");

    // INIT → RUNNING
    // No prerequisites — should succeed on first attempt.  Retry guards against
    // a momentary mutex timeout inside advance().
    while (!StateMachine::isInState("led_control", stateToString(LedControlState::RUNNING))) {
        StateMachine::advance("led_control");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }
    ESP_LOGI(TAG, "LED running");

    // Permanent idle loop — this task has no periodic work; it is kept alive
    // because shelfbot.cpp stores its handle for potential future vTaskDelete.
    // There is intentionally no vTaskDelete(nullptr) here: the loop never exits.
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ---------------------------------------------------------------------------
// Public API — shelfbot.cpp calls led_control_setup() then creates the task.
// ---------------------------------------------------------------------------
void led_control_setup() {
    ESP_LOGI(TAG, "LED setup — task will be created by shelfbot");
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
