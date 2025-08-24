#include "motor_control.h"
#include "driver/gpio.h" // For gpio_pad_select_gpio

static const char* TAG = "motor_control";

// Global stepper motor objects and configuration
static FastAccelStepperEngine engine = FastAccelStepperEngine();
static FastAccelStepper* steppers[NUM_MOTORS] = {nullptr};
static const int motorPins[NUM_MOTORS][2] = {
    {27, 26}, {14, 12}, {13, 15}, {4, 16}, {17, 5}, {18, 19}
};

// Conversion factor from radians to steps. This is the core of the unit standardization.
const double RADS_TO_STEPS = (double)(STEPS_PER_REVOLUTION * MICROSTEPPING * GEAR_RATIO) / (2.0 * M_PI);

void motor_control_begin() {
    ESP_LOGI(TAG, "Initializing motor system");
    engine.init();
    
    for (int i = 0; i < NUM_MOTORS; i++) {
        // --- FIX: Forcefully configure pins as GPIO ---
        // This is crucial for strapping pins (like 15, 12, 5, 4) to ensure they
        // are released from their boot-time functions and can be controlled by peripherals.
        gpio_pad_select_gpio((gpio_num_t)motorPins[i][0]); // PULSE Pin
        gpio_pad_select_gpio((gpio_num_t)motorPins[i][1]); // DIR Pin

        steppers[i] = engine.stepperConnectToPin(motorPins[i][0]);
        if (steppers[i]) {
            steppers[i]->setDirectionPin(motorPins[i][1], true);

            steppers[i]->setAutoEnable(true);
            steppers[i]->setSpeedInHz(4000); // Default speed
            steppers[i]->setAcceleration(2000); // Default acceleration
            steppers[i]->setCurrentPosition(0);
        }
    }
    // NOTE: A background task is not needed. The FastAccelStepper library uses hardware interrupts
    // on the ESP32 to generate steps in the background, so no polling `engine.run()` is required.
}

// --- ROS 2 Standard Interface Implementation ---

void motor_control_set_position(uint8_t index, double position_rad) {
    if (index >= NUM_MOTORS || !steppers[index]) return;
    ESP_LOGI(TAG, "Conversion Info: STEPS_PER_REV=%.1f, MICROSTEPPING=%.1f, GEAR_RATIO=%.1f, RADS_TO_STEPS=%.4f", STEPS_PER_REVOLUTION, MICROSTEPPING, GEAR_RATIO, RADS_TO_STEPS);
    long target_steps = (long)(position_rad * RADS_TO_STEPS);
    ESP_LOGI(TAG, "Motor %d: Converting %.2f rad to %ld steps", index, position_rad, target_steps);
    ESP_LOGI(TAG, "Executing: steppers[%d]->moveTo(%ld)", index, target_steps);
    steppers[index]->moveTo(target_steps);
}

double motor_control_get_position(uint8_t index) {
    if (index >= NUM_MOTORS || !steppers[index]) return 0.0;
    return (double)steppers[index]->getCurrentPosition() / RADS_TO_STEPS;
}

double motor_control_get_velocity(uint8_t index) {
    if (index >= NUM_MOTORS || !steppers[index]) return 0.0;
    // Library returns speed in mHz (steps/1000s). Convert to steps/s, then to rad/s.
    double steps_per_sec = (double)steppers[index]->getCurrentSpeedInMilliHz() / 1000.0;
    return steps_per_sec / RADS_TO_STEPS;
}

// --- Add this new function ---
void motor_control_set_velocity(uint8_t index, double velocity_rad_s) {
    if (index >= NUM_MOTORS || !steppers[index]) return;

    // If velocity is very close to zero, stop the motor.
    if (fabs(velocity_rad_s) < 1e-4) {
        steppers[index]->stopMove();
        return;
    }

    // Convert velocity in rad/s to speed in Hz (steps/s)
    long speed_hz = (long)fabs(velocity_rad_s * RADS_TO_STEPS);

    // Set the motor speed and acceleration
    steppers[index]->setSpeedInHz(speed_hz);
    steppers[index]->setAcceleration(speed_hz / 2); // Use a reasonable acceleration

    // Set the direction and command the motor to run continuously
    if (velocity_rad_s > 0) {
        steppers[index]->runForward();
    } else {
        steppers[index]->runBackward();
    }
}

// --- Utility Function Implementations ---

void motor_control_set_speed_hz(uint8_t index, long speed_hz) {
    if (index >= NUM_MOTORS || !steppers[index]) return;
    steppers[index]->setSpeedInHz(speed_hz);
    steppers[index]->setAcceleration(speed_hz / 2); // Set a reasonable default acceleration
}

void motor_control_set_all_speeds_hz(long speed_hz) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_control_set_speed_hz(i, speed_hz);
    }
}

bool motor_control_is_motor_running(uint8_t index) {
    if (index >= NUM_MOTORS || !steppers[index]) return false;
    return steppers[index]->isRunning();
}

void motor_control_stop_motor(uint8_t index) {
    if (index >= NUM_MOTORS || !steppers[index]) return;
    ESP_LOGI(TAG, "Motor %d: Stopping motor", index);
    int pulse_pin = motorPins[index][0];
    int dir_pin = motorPins[index][1];
    int dir_level = gpio_get_level((gpio_num_t)dir_pin);
    ESP_LOGI(TAG, "Motor %d GPIOs: PULSE_PIN=%d, DIR_PIN=%d, DIR_LEVEL=%d", index, pulse_pin, dir_pin, dir_level);
    steppers[index]->forceStop();
}

void motor_control_stop_all_motors() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (steppers[i]) steppers[i]->forceStop();
    }
}

// --- DEPRECATED Function Implementations (for REST API) ---
// These now act as wrappers around the new radian-based API.

void motor_control_set_motor_position_double(uint8_t index, double position_deg) {
    // Use direct expression to avoid macro collision from Arduino.h
    motor_control_set_position(index, position_deg * (M_PI / 180.0));
}

double motor_control_get_motor_position_double(uint8_t index) {
    // Use direct expression to avoid macro collision from Arduino.h
    return motor_control_get_position(index) * (180.0 / M_PI);
}

double motor_control_get_motor_velocity_double(uint8_t index) {
    // Use direct expression to avoid macro collision from Arduino.h
    return motor_control_get_velocity(index) * (180.0 / M_PI);
}

bool motor_control_move_all_motors_vector(const double* positions_deg, size_t num_positions, long speed, bool non_blocking) {
    if (!positions_deg || num_positions > NUM_MOTORS) return false;
    motor_control_set_all_speeds_hz(speed);
    for (size_t i = 0; i < num_positions; i++) {
        motor_control_set_motor_position_double(i, positions_deg[i]);
    }
    // Note: Non-blocking logic would need to be implemented if required for the REST API.
    return true;
}
