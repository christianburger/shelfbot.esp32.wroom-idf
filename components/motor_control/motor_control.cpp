#include <motor_control.hpp>
#include <state_machine.hpp>
#include <state_machine_lifecycle.hpp>

static const char* TAG = "MotorControl";
static TaskHandle_t update_task_handle = nullptr;
static bool stop_requested = false;

// Internal state storage (5 motors)
static float motor_positions[NUM_MOTORS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static float motor_velocities[NUM_MOTORS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static bool motor_running[NUM_MOTORS] = {false, false, false, false, false};

static void set_motor_state(MotorControlState new_state) {
    StateMachine::changeState("motor_control", stateToString(new_state));
}

static void motor_update_task(void* arg) {
    while (true) {
        if (stop_requested) {
            set_motor_state(MotorControlState::STOPPED);
            vTaskDelay(pdMS_TO_TICKS(MOTOR_STOP_TO_SETUP_DELAY_MS));
            stop_requested = false;
            set_motor_state(MotorControlState::SETUP);
            motor_control_setup();
            motor_control_init();
            motor_control_start();
        }
        // In RUNNING state: perform periodic motor control tasks
        // (e.g., apply velocities, update encoders)
        // For now, just mark motors as running if they have non‑zero velocity.
        for (int i = 0; i < NUM_MOTORS; ++i) {
            if (motor_velocities[i] != 0.0f) {
                motor_running[i] = true;
            } else {
                motor_running[i] = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void motor_control_setup() {
    ESP_LOGI(TAG, "Motor setup");
    set_motor_state(MotorControlState::SETUP);
}

void motor_control_init() {
    ESP_LOGI(TAG, "Motor init");
    // Hardware initialisation here if any
    for (int i = 0; i < NUM_MOTORS; ++i) {
        motor_positions[i] = 0.0f;
        motor_velocities[i] = 0.0f;
        motor_running[i] = false;
    }
    set_motor_state(MotorControlState::INIT);
}

void motor_control_start() {
    if (update_task_handle) return;
    stop_requested = false;
    set_motor_state(MotorControlState::RUNNING);
    xTaskCreate(motor_update_task, "motor_update", 4096, nullptr, 5, &update_task_handle);
    ESP_LOGI(TAG, "Motor started");
}

void motor_control_stop() {
    stop_requested = true;
    ESP_LOGI(TAG, "Motor stop requested");
}

void motor_control_update() {
    // Called from main loop – optional
}

void motor_control_begin() {
    motor_control_setup();
    motor_control_init();
    motor_control_start();
}

void motor_control_set_position(uint8_t motor, float position) {
    if (motor < NUM_MOTORS) {
        motor_positions[motor] = position;
        // Send to hardware driver (placeholder)
        ESP_LOGD(TAG, "Set motor %d position %.2f", motor, position);
    }
}

void motor_control_set_velocity(uint8_t motor, float velocity) {
    if (motor < NUM_MOTORS) {
        motor_velocities[motor] = velocity;
        // Send to hardware driver (placeholder)
        ESP_LOGD(TAG, "Set motor %d velocity %.2f", motor, velocity);
    }
}

float motor_control_get_position(uint8_t motor) {
    if (motor < NUM_MOTORS) {
        return motor_positions[motor];
    }
    return 0.0f;
}

// ---------------------------------------------------------------------------
// Additional functions required by HTTP server
// ---------------------------------------------------------------------------
float motor_control_get_velocity(uint8_t motor) {
    if (motor < NUM_MOTORS) {
        return motor_velocities[motor];
    }
    return 0.0f;
}

bool motor_control_is_motor_running(uint8_t motor) {
    if (motor < NUM_MOTORS) {
        return motor_running[motor];
    }
    return false;
}

void motor_control_apply(uint8_t motor, float position, float velocity) {
    if (motor < NUM_MOTORS) {
        motor_control_set_position(motor, position);
        motor_control_set_velocity(motor, velocity);
        // Optionally trigger immediate update
        ESP_LOGI(TAG, "Applied motor %d: pos=%.2f, vel=%.2f", motor, position, velocity);
    }
}