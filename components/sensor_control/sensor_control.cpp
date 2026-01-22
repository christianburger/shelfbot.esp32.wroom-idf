#include "sensor_control.hpp"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_timer.h"

static const char* TAG = "sensor_control";

// --- Configuration ---
const float COLLISION_THRESHOLD_CM = 20.0;
#define SENSOR_TIMEOUT_US 30000

// --- Globals for Driver ---
QueueHandle_t distance_data_queue;
QueueHandle_t motor_stop_queue;
static SemaphoreHandle_t measurement_done_sem;
static gptimer_handle_t shared_timeout_timer = NULL;
static volatile int active_sensor_index = -1;

// FIXED: Changed GPIO 0 to GPIO 5 to avoid boot mode conflict
static const int sensor_pins[NUM_SENSORS][2] = {
    {32, 34},  // Sensor 0: TRIG=GPIO32, ECHO=GPIO34
    {5, 35}    // Sensor 1: TRIG=GPIO5,  ECHO=GPIO35 (FIXED from GPIO 0)
};

// --- State Machine and Data ---
typedef enum {
    SENSOR_IDLE,
    SENSOR_TRIGGERED,
    SENSOR_MEASURING
} sensor_state_t;

typedef struct {
    gpio_num_t trig_pin;
    gpio_num_t echo_pin;
    volatile sensor_state_t state;
    volatile int64_t trigger_time;
    volatile int64_t start_time;
    volatile uint32_t pulse_duration;
    volatile float distance;
} sensor_control_t;

static sensor_control_t sensors[NUM_SENSORS];

// --- Interrupt Service Routines (ISRs) ---

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    int sensor_index = (int)arg;
    sensor_control_t* sensor = &sensors[sensor_index];
    int64_t now = esp_timer_get_time();

    if (sensor->state == SENSOR_TRIGGERED && gpio_get_level(sensor->echo_pin) == 1) {
        sensor->start_time = now;
        sensor->state = SENSOR_MEASURING;
    } else if (sensor->state == SENSOR_MEASURING && gpio_get_level(sensor->echo_pin) == 0) {
        gptimer_stop(shared_timeout_timer);
        sensor->pulse_duration = now - sensor->start_time;
        sensor->state = SENSOR_IDLE;
        BaseType_t task_woken = pdFALSE;
        xSemaphoreGiveFromISR(measurement_done_sem, &task_woken);
    }
}

static bool IRAM_ATTR timer_isr_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    gptimer_stop(shared_timeout_timer);
    if (active_sensor_index != -1) {
        sensors[active_sensor_index].pulse_duration = SENSOR_TIMEOUT_US;
        sensors[active_sensor_index].state = SENSOR_IDLE;
    }
    BaseType_t task_woken = pdFALSE;
    xSemaphoreGiveFromISR(measurement_done_sem, &task_woken);
    return false;
}

// --- FreeRTOS Task ---
static void sensor_reader_task(void* arg) {
    float distances[NUM_SENSORS];
    for (;;) {
        bool obstacle_detected = false;
        for (int i = 0; i < NUM_SENSORS; i++) {
            sensors[i].state = SENSOR_TRIGGERED;
            active_sensor_index = i;

            sensors[i].trigger_time = esp_timer_get_time();
            gpio_set_level(sensors[i].trig_pin, 1);
            esp_rom_delay_us(10);
            gpio_set_level(sensors[i].trig_pin, 0);

            gptimer_set_raw_count(shared_timeout_timer, 0);
            gptimer_start(shared_timeout_timer);

            if (xSemaphoreTake(measurement_done_sem, pdMS_TO_TICKS(50)) == pdTRUE) {
                // Measurement completed
            } else {
                gptimer_stop(shared_timeout_timer);
                sensors[i].state = SENSOR_IDLE;
                sensors[i].pulse_duration = SENSOR_TIMEOUT_US;
            }

            distances[i] = (float)sensors[i].pulse_duration;
            if (distances[i] > 0 && distances[i] < COLLISION_THRESHOLD_CM) {
                obstacle_detected = true;
            }
        }
        active_sensor_index = -1;

        xQueueOverwrite(distance_data_queue, &distances);
        if (obstacle_detected) {
            int stop_signal = 1;
            xQueueSend(motor_stop_queue, &stop_signal, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// --- Public API Implementation ---
void sensor_control_init() {
    ESP_LOGI(TAG, "Initializing sensor control component (GPIO 5 fix applied)");
    distance_data_queue = xQueueCreate(1, sizeof(float[NUM_SENSORS]));
    motor_stop_queue = xQueueCreate(1, sizeof(int));
    measurement_done_sem = xSemaphoreCreateBinary();

    // --- Configure Shared Timer ---
    gptimer_config_t timer_config = {};
    timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timer_config.direction = GPTIMER_COUNT_UP;
    timer_config.resolution_hz = 1000000;

    gptimer_new_timer(&timer_config, &shared_timeout_timer);

    gptimer_alarm_config_t alarm_config = {};
    alarm_config.alarm_count = SENSOR_TIMEOUT_US;
    alarm_config.reload_count = 0;

    gptimer_set_alarm_action(shared_timeout_timer, &alarm_config);

    gptimer_event_callbacks_t cbs = { .on_alarm = timer_isr_handler };
    gptimer_register_event_callbacks(shared_timeout_timer, &cbs, NULL);
    gptimer_enable(shared_timeout_timer);

    // --- Configure GPIOs ---
    gpio_install_isr_service(0);
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensors[i].trig_pin = (gpio_num_t)sensor_pins[i][0];
        sensors[i].echo_pin = (gpio_num_t)sensor_pins[i][1];
        sensors[i].state = SENSOR_IDLE;

        gpio_config_t trig_io_conf = {};
        trig_io_conf.pin_bit_mask = (1ULL << sensors[i].trig_pin);
        trig_io_conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&trig_io_conf);
        gpio_set_level(sensors[i].trig_pin, 0);

        gpio_config_t echo_io_conf = {};
        echo_io_conf.pin_bit_mask = (1ULL << sensors[i].echo_pin);
        echo_io_conf.mode = GPIO_MODE_INPUT;
        echo_io_conf.intr_type = GPIO_INTR_ANYEDGE;
        gpio_config(&echo_io_conf);
        gpio_isr_handler_add(sensors[i].echo_pin, gpio_isr_handler, (void*)i);
    }
    ESP_LOGI(TAG, "Sensor driver initialized with GPIO 5 fix");
}

void sensor_control_start_task() {
    xTaskCreate(sensor_reader_task, "sensor_reader", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Sensor reader task started");
}
