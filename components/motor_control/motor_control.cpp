#include <motor_control.hpp>
#include <state_machine.hpp>
#include <state_machine_lifecycle.hpp>

static const char* TAG = "MotorControl";
static TaskHandle_t update_task_handle = nullptr;

static float motor_positions[NUM_MOTORS]  = {};
static float motor_velocities[NUM_MOTORS] = {};
static bool  motor_running[NUM_MOTORS]    = {};

// ---------------------------------------------------------------------------
// Internal update task – runs while module is RUNNING.
// ---------------------------------------------------------------------------
static void motor_update_task(void* /*arg*/) {
    while (true) {
        for (int i = 0; i < NUM_MOTORS; ++i)
            motor_running[i] = (motor_velocities[i] != 0.0f);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ---------------------------------------------------------------------------
// Self-driving lifecycle task
// ---------------------------------------------------------------------------
static void motor_lifecycle_task(void* /*arg*/) {
    static const uint32_t RETRY_MS = 1000;

    // setup -> init
    while (!StateMachine::canTransition("motor_control",
                                        stateToString(MotorControlState::INIT))) {
        ESP_LOGD(TAG, "waiting for prereqs: setup->init");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }
    // Hardware init: reset arrays
    for (int i = 0; i < NUM_MOTORS; ++i) {
        motor_positions[i]  = 0.0f;
        motor_velocities[i] = 0.0f;
        motor_running[i]    = false;
    }
    StateMachine::changeState("motor_control", stateToString(MotorControlState::INIT));
    ESP_LOGI(TAG, "Motor init done");

    // init -> running
    while (!StateMachine::canTransition("motor_control",
                                        stateToString(MotorControlState::RUNNING))) {
        ESP_LOGD(TAG, "waiting for prereqs: init->running");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }
    xTaskCreate(motor_update_task, "motor_update", 2048, nullptr, 5, &update_task_handle);
    StateMachine::changeState("motor_control", stateToString(MotorControlState::RUNNING));
    ESP_LOGI(TAG, "Motor running");

    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// Public lifecycle API
// ---------------------------------------------------------------------------
void motor_control_setup() {
    ESP_LOGI(TAG, "Motor setup: spawning lifecycle task");
    xTaskCreate(motor_lifecycle_task, "motor_lifecycle", 2048, nullptr, 3, nullptr);
}

void motor_control_init()   {}  // driven by lifecycle task
void motor_control_start()  {}  // driven by lifecycle task
void motor_control_stop() {
    if (update_task_handle) {
        vTaskDelete(update_task_handle);
        update_task_handle = nullptr;
    }
    for (int i = 0; i < NUM_MOTORS; ++i) {
        motor_velocities[i] = 0.0f;
        motor_running[i]    = false;
    }
    StateMachine::changeState("motor_control",
                               stateToString(MotorControlState::STOPPED),
                               /*force=*/true);
}
void motor_control_update() {}
void motor_control_begin()  { motor_control_setup(); }

// ---------------------------------------------------------------------------
// Runtime API
// ---------------------------------------------------------------------------
void motor_control_set_position(uint8_t motor, float position) {
    if (motor < NUM_MOTORS) motor_positions[motor] = position;
}

void motor_control_set_velocity(uint8_t motor, float velocity) {
    if (motor < NUM_MOTORS) motor_velocities[motor] = velocity;
}

float motor_control_get_position(uint8_t motor) {
    return (motor < NUM_MOTORS) ? motor_positions[motor] : 0.0f;
}

float motor_control_get_velocity(uint8_t motor) {
    return (motor < NUM_MOTORS) ? motor_velocities[motor] : 0.0f;
}

bool motor_control_is_motor_running(uint8_t motor) {
    return (motor < NUM_MOTORS) && motor_running[motor];
}

void motor_control_apply(uint8_t motor, float position, float velocity) {
    if (motor < NUM_MOTORS) {
        motor_positions[motor]  = position;
        motor_velocities[motor] = velocity;
        ESP_LOGI(TAG, "Applied motor %d: pos=%.2f vel=%.2f", motor, position, velocity);
    }
}