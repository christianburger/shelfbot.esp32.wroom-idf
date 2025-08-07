#include "motor_control.h"
#include "FastAccelStepper.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "motor_control";

// Global stepper motor objects and configuration
static FastAccelStepperEngine engine = FastAccelStepperEngine();
static FastAccelStepper* steppers[NUM_MOTORS] = {nullptr};
static const int motorPins[NUM_MOTORS][2] = {
    {27, 26}, {14, 12}, {13, 15}, {4, 16}, {17, 5}, {18, 19}
};
static std::vector<long> motorSpeeds(NUM_MOTORS, 4000);

void motor_control_begin() {
    ESP_LOGI(TAG, "Initializing motor system");
    engine.init();
    
    for (int i = 0; i < NUM_MOTORS; i++) {
        steppers[i] = engine.stepperConnectToPin(motorPins[i][0]);
        if (steppers[i]) {
            steppers[i]->setDirectionPin(motorPins[i][1]);
            steppers[i]->setAutoEnable(true);
            steppers[i]->setSpeedInHz(motorSpeeds[i]);
            steppers[i]->setAcceleration(motorSpeeds[i] / 2);
            steppers[i]->setCurrentPosition(0);
            ESP_LOGI(TAG, "Motor %d initialized with speed %ld", i, motorSpeeds[i]);
        } else {
            ESP_LOGE(TAG, "Failed to initialize motor %d", i);
        }
    }
}

long motor_control_get_motor_speed(uint8_t index) {
    if (index >= NUM_MOTORS) return 0;
    return motorSpeeds[index];
}

void motor_control_set_motor_speed(uint8_t index, long speed) {
    if (index >= NUM_MOTORS) return;
    motorSpeeds[index] = speed;
    if (steppers[index]) {
        steppers[index]->setSpeedInHz(speed);
        steppers[index]->setAcceleration(speed / 2);
    }
}

void motor_control_set_all_motor_speeds(long speed) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_control_set_motor_speed(i, speed);
    }
}

bool motor_control_are_all_motors_stopped() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (steppers[i] && steppers[i]->isRunning()) {
            return false;
        }
    }
    return true;
}

void motor_control_move_all_motors(long position, long speed, bool nonBlocking) {
    motor_control_set_all_motor_speeds(speed);
    for (int i = 0; i < NUM_MOTORS; i++) {
        if(steppers[i]) {
            steppers[i]->moveTo(position);
        }
    }

    if (!nonBlocking) {
        while (!motor_control_are_all_motors_stopped()) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

bool motor_control_move_all_motors_vector(const double* positions, size_t num_positions, long speed, bool nonBlocking) {
    if (!positions || num_positions != NUM_MOTORS) return false;
    motor_control_set_all_motor_speeds(speed);
    for (size_t i = 0; i < num_positions; i++) {
        if (steppers[i]) {
            steppers[i]->moveTo(static_cast<long>(positions[i]));
        }
    }

    if (!nonBlocking) {
        while (!motor_control_are_all_motors_stopped()) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    return true;
}

void motor_control_set_motor_position_double(uint8_t index, double position) {
    if (index >= NUM_MOTORS || !steppers[index]) return;
    double conversion_factor = (GEAR_RATIO * MICROSTEPPING * STEPS_PER_REVOLUTION) / 360.0;
    long total_steps = (long)(position * conversion_factor);
    steppers[index]->moveTo(total_steps);
}

double motor_control_get_motor_position_double(uint8_t index) {
    if (index >= NUM_MOTORS || !steppers[index]) return 0.0;
    return static_cast<double>(steppers[index]->getCurrentPosition());
}

double motor_control_get_motor_velocity_double(uint8_t index) {
    if (index >= NUM_MOTORS || !steppers[index]) return 0.0;
    return static_cast<double>(steppers[index]->getCurrentSpeedInUs());
}

void motor_control_stop_motor(uint8_t index) {
    if (index >= NUM_MOTORS || !steppers[index]) return;
    steppers[index]->forceStop();
}

void motor_control_stop_all_motors() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (steppers[i]) {
            steppers[i]->forceStop();
        }
    }
}

bool motor_control_is_motor_running(uint8_t index) {
    if (index >= NUM_MOTORS || !steppers[index]) return false;
    return steppers[index]->isRunning();
}
