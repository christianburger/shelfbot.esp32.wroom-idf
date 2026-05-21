#include <motor_control.hpp>

static auto TAG = "motor_control";

static FastAccelStepperEngine engine;
static FastAccelStepper* steppers[NUM_MOTORS] = {nullptr};
static int8_t motor_direction_sign[NUM_MOTORS] = {0};

static const int motorPins[NUM_MOTORS][2] = {
    {27, 26},  // Motor 0: PULSE=GPIO27, DIR=GPIO26
    {14, 33},  // Motor 1: PULSE=GPIO14, DIR=GPIO33
    {13, 19},  // Motor 2: PULSE=GPIO13, DIR=GPIO19
    { 4, 18},  // Motor 3: PULSE=GPIO4,  DIR=GPIO18
    {12, 23},  // Motor 4: PULSE=GPIO12, DIR=GPIO23
};

// Radians → steps conversion factor
const double RADS_TO_STEPS =
    (STEPS_PER_REVOLUTION * MICROSTEPPING * GEAR_RATIO) / (2.0 * M_PI);

// Default speed used when a position command is issued with velocity == 0
static constexpr long DEFAULT_SPEED_HZ   = 4000;
static constexpr long DEFAULT_ACCEL_HZ_S = 2000;
// Minimum velocity before we treat it as "zero"
static constexpr double VEL_DEADBAND = 1e-4;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static inline long vel_to_hz(double velocity_rad_s) {
    return static_cast<long>(fabs(velocity_rad_s) * RADS_TO_STEPS);
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void motor_control_begin() {
    ESP_LOGI(TAG, "Initializing motor system (RADS_TO_STEPS=%.4f)", RADS_TO_STEPS);
    engine.init();

    for (int i = 0; i < NUM_MOTORS; i++) {
        gpio_pad_select_gpio(static_cast<gpio_num_t>(motorPins[i][0]));
        gpio_pad_select_gpio(static_cast<gpio_num_t>(motorPins[i][1]));

        steppers[i] = engine.stepperConnectToPin(motorPins[i][0]);
        if (steppers[i]) {
            steppers[i]->setDirectionPin(motorPins[i][1], true);
            steppers[i]->setAutoEnable(true);
            steppers[i]->setSpeedInHz(DEFAULT_SPEED_HZ);
            steppers[i]->setAcceleration(DEFAULT_ACCEL_HZ_S);
            steppers[i]->setCurrentPosition(0);
            motor_direction_sign[i] = 0;
            ESP_LOGI(TAG, "  Motor %d OK (PULSE=%d DIR=%d)", i,
                     motorPins[i][0], motorPins[i][1]);
        } else {
            ESP_LOGE(TAG, "  Motor %d FAILED to connect", i);
        }
    }
}

// ---------------------------------------------------------------------------
// Primary unified command
//
//  velocity == 0              → stop
//  position == 0, vel != 0   → run continuously in velocity direction
//  position != 0, vel != 0   → move to position at |velocity| speed
//  position != 0, vel == 0   → move to position at default speed
// ---------------------------------------------------------------------------
void motor_control_apply(const uint8_t index,
                         const double   position_rad,
                         const double   velocity_rad_s) {
    if (index >= NUM_MOTORS || !steppers[index]) return;

    const bool has_velocity = fabs(velocity_rad_s) >= VEL_DEADBAND;
    const bool has_position = fabs(position_rad)   >= VEL_DEADBAND;

    // --- Stop ---
    if (!has_velocity && !has_position) {
        steppers[index]->stopMove();
        motor_direction_sign[index] = 0;
        return;
    }

    // Determine speed for this command
    const long speed_hz = has_velocity ? vel_to_hz(velocity_rad_s) : DEFAULT_SPEED_HZ;
    if (speed_hz < 1) {
        steppers[index]->stopMove();
        motor_direction_sign[index] = 0;
        return;
    }

    steppers[index]->setSpeedInHz(speed_hz);
    steppers[index]->setAcceleration(std::max(1L, speed_hz / 2));

    // --- Continuous velocity (no position target) ---
    if (!has_position) {
        if (velocity_rad_s > 0) {
            steppers[index]->runForward();
            motor_direction_sign[index] = 1;
        } else {
            steppers[index]->runBackward();
            motor_direction_sign[index] = -1;
        }
        ESP_LOGI(TAG, "Motor %d: continuous %.4f rad/s (%ld Hz)",
                 index, velocity_rad_s, speed_hz);
        return;
    }

    // --- Move to position ---
    const long target_steps = static_cast<long>(position_rad * RADS_TO_STEPS);
    const long current_steps = steppers[index]->getCurrentPosition();
    if (target_steps > current_steps) {
        motor_direction_sign[index] = 1;
    } else if (target_steps < current_steps) {
        motor_direction_sign[index] = -1;
    } else {
        motor_direction_sign[index] = 0;
    }
    steppers[index]->moveTo(target_steps);
    ESP_LOGI(TAG, "Motor %d: moveTo %.4f rad (%ld steps) at %ld Hz",
             index, position_rad, target_steps, speed_hz);
}

// ---------------------------------------------------------------------------
// Individual setters (thin wrappers around motor_control_apply)
// ---------------------------------------------------------------------------
void motor_control_set_velocity(const uint8_t index, const double velocity_rad_s) {
    // Position stays unchanged: pass 0 to select continuous-velocity mode
    motor_control_apply(index, 0.0, velocity_rad_s);
}

void motor_control_set_position(const uint8_t index, const double position_rad) {
    if (index >= NUM_MOTORS || !steppers[index]) return;

    const long target_steps = static_cast<long>(position_rad * RADS_TO_STEPS);
    const long current_steps = steppers[index]->getCurrentPosition();
    if (target_steps > current_steps) {
        motor_direction_sign[index] = 1;
    } else if (target_steps < current_steps) {
        motor_direction_sign[index] = -1;
    } else {
        motor_direction_sign[index] = 0;
    }

    steppers[index]->setSpeedInHz(DEFAULT_SPEED_HZ);
    steppers[index]->setAcceleration(DEFAULT_ACCEL_HZ_S);
    steppers[index]->moveTo(target_steps);
}

// ---------------------------------------------------------------------------
// Getters — both return signed values
// ---------------------------------------------------------------------------
double motor_control_get_position(const uint8_t index) {
    if (index >= NUM_MOTORS || !steppers[index]) return 0.0;
    return static_cast<double>(steppers[index]->getCurrentPosition()) / RADS_TO_STEPS;
}

double motor_control_get_velocity(const uint8_t index) {
    if (index >= NUM_MOTORS || !steppers[index]) return 0.0;

    // getCurrentSpeedInMilliHz() is always positive — derive sign from direction
    const double steps_per_sec =
        static_cast<double>(steppers[index]->getCurrentSpeedInMilliHz()) / 1000.0;

    const int sign = (motor_direction_sign[index] != 0) ? motor_direction_sign[index] : 1;

    return (steps_per_sec * sign) / RADS_TO_STEPS;
}

// ---------------------------------------------------------------------------
// Utility
// ---------------------------------------------------------------------------
void motor_control_set_speed_hz(const uint8_t index, const long speed_hz) {
    if (index >= NUM_MOTORS || !steppers[index]) return;
    steppers[index]->setSpeedInHz(speed_hz);
    steppers[index]->setAcceleration(std::max(1L, speed_hz / 2));
}

void motor_control_set_all_speeds_hz(const long speed_hz) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_control_set_speed_hz(i, speed_hz);
    }
}

bool motor_control_is_motor_running(const uint8_t index) {
    if (index >= NUM_MOTORS || !steppers[index]) return false;
    return steppers[index]->isRunning();
}

void motor_control_stop_motor(const uint8_t index) {
    if (index >= NUM_MOTORS || !steppers[index]) return;
    ESP_LOGI(TAG, "Motor %d: forceStop (PULSE=%d DIR=%d level=%d)",
             index, motorPins[index][0], motorPins[index][1],
             gpio_get_level(static_cast<gpio_num_t>(motorPins[index][1])));
    steppers[index]->forceStop();
    motor_direction_sign[index] = 0;
}

void motor_control_stop_all_motors() {
    for (auto& s : steppers) {
        if (s) s->forceStop();
    }
    for (auto& dir : motor_direction_sign) {
        dir = 0;
    }
}

// ---------------------------------------------------------------------------
// DEPRECATED — degree-based wrappers for REST API
// ---------------------------------------------------------------------------
void motor_control_set_motor_position_double(const uint8_t index, const double position_deg) {
    motor_control_set_position(index, position_deg * (M_PI / 180.0));
}

double motor_control_get_motor_position_double(const uint8_t index) {
    return motor_control_get_position(index) * (180.0 / M_PI);
}

double motor_control_get_motor_velocity_double(const uint8_t index) {
    return motor_control_get_velocity(index) * (180.0 / M_PI);
}

bool motor_control_move_all_motors_vector(const double* positions,
                                          const size_t  num_positions,
                                          const long    speed,
                                          bool          non_blocking) {
    if (!positions || num_positions > NUM_MOTORS) return false;
    motor_control_set_all_speeds_hz(speed);
    for (size_t i = 0; i < num_positions; i++) {
        motor_control_set_motor_position_double(i, positions[i]);
    }
    return true;
}
