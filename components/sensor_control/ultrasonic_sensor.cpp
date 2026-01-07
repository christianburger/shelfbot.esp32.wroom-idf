// Corrected ultrasonic_sensor.cpp
#include "ultrasonic_sensor.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_timer.h"
#include <vector>
#include <cstring>

static const char* TAG = "Ultrasonic_Sensor";

// Globals for queues
QueueHandle_t distance_data_queue;
QueueHandle_t motor_stop_queue;

// Shared resources for ISRs
static gptimer_handle_t shared_timeout_timer = NULL;
static SemaphoreHandle_t measurement_done_sem = NULL;
static volatile int active_sensor_index = -1;
static UltrasonicSensorArray* g_array = nullptr;  // Global for ISRs to access configs

// ISR handlers
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    int sensor_index = (int)arg;
    int64_t now = esp_timer_get_time();
    UltrasonicSensorConfig& config = g_array->configs_[sensor_index];

    if (config.state == SENSOR_TRIGGERED && gpio_get_level(config.echo_pin) == 1) {
        config.start_time = now;
        config.state = SENSOR_MEASURING;
    } else if (config.state == SENSOR_MEASURING && gpio_get_level(config.echo_pin) == 0) {
        gptimer_stop(shared_timeout_timer);
        config.pulse_duration = now - config.start_time;
        config.state = SENSOR_IDLE;
        BaseType_t task_woken = pdFALSE;
        xSemaphoreGiveFromISR(measurement_done_sem, &task_woken);
        if (task_woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

static bool IRAM_ATTR timer_isr_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    gptimer_stop(shared_timeout_timer);
    if (active_sensor_index != -1) {
        UltrasonicSensorConfig& config = g_array->configs_[active_sensor_index];
        config.pulse_duration = config.timeout_us;
        config.state = SENSOR_IDLE;
        BaseType_t task_woken = pdFALSE;
        xSemaphoreGiveFromISR(measurement_done_sem, &task_woken);
        if (task_woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
    return true;
}

// ============================================================================
// UltrasonicSensorArray Implementation
// ============================================================================

UltrasonicSensorArray::UltrasonicSensorArray(uint8_t num_sensors)
    : num_sensors_(num_sensors)
    , initialized_(false)
    , array_mutex_(nullptr)
{
    array_mutex_ = xSemaphoreCreateMutex();
}

UltrasonicSensorArray::~UltrasonicSensorArray() {
    if (array_mutex_) {
        vSemaphoreDelete(array_mutex_);
    }
}

bool UltrasonicSensorArray::init() {
    if (initialized_) return true;

    gptimer_config_t timer_config = {};
    timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timer_config.direction = GPTIMER_COUNT_UP;
    timer_config.resolution_hz = 1000000; // 1MHz, 1 tick = 1us

    esp_err_t err = gptimer_new_timer(&timer_config, &shared_timeout_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(err));
        return false;
    }

    gptimer_event_callbacks_t cbs = {.on_alarm = timer_isr_handler};
    err = gptimer_register_event_callbacks(shared_timeout_timer, &cbs, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register callbacks: %s", esp_err_to_name(err));
        return false;
    }

    err = gptimer_enable(shared_timeout_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable timer: %s", esp_err_to_name(err));
        return false;
    }

    gpio_install_isr_service(0);

    measurement_done_sem = xSemaphoreCreateBinary();
    if (measurement_done_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return false;
    }

    initialized_ = true;
    return true;
}

bool UltrasonicSensorArray::add_sensor(uint8_t index, const UltrasonicSensorConfig& sensor_config) {
    if (index >= num_sensors_) {
        ESP_LOGE(TAG, "Index out of bounds");
        return false;
    }

    configs_[index] = sensor_config;

    // Init pins
    gpio_config_t trig_io_conf = {};
    trig_io_conf.pin_bit_mask = (1ULL << configs_[index].trig_pin);
    trig_io_conf.mode = GPIO_MODE_OUTPUT;
    trig_io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    trig_io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    trig_io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&trig_io_conf);
    gpio_set_level(configs_[index].trig_pin, 0);

    gpio_config_t echo_io_conf = {};
    echo_io_conf.pin_bit_mask = (1ULL << configs_[index].echo_pin);
    echo_io_conf.mode = GPIO_MODE_INPUT;
    echo_io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    echo_io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    echo_io_conf.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&echo_io_conf);

    // Add ISR
    gpio_isr_handler_add(configs_[index].echo_pin, gpio_isr_handler, (void*)(intptr_t)index);

    return true;
}

bool UltrasonicSensorArray::read_all_single(std::vector<SensorCommon::Reading>& readings, uint32_t timeout_ms) {
    if (!initialized_) return false;

    readings.resize(num_sensors_);
    bool all_success = true;
    bool obstacle_detected = false;

    g_array = this;  // Set global for ISRs

    for (uint8_t i = 0; i < num_sensors_; i++) {
        UltrasonicSensorConfig& config = configs_[i];
        config.state = SENSOR_TRIGGERED;
        active_sensor_index = i;

        // Trigger
        gpio_set_level(config.trig_pin, 1);
        esp_rom_delay_us(10);
        gpio_set_level(config.trig_pin, 0);

        // Set alarm for this sensor's timeout
        gptimer_alarm_config_t alarm_config = {};
        alarm_config.alarm_count = config.timeout_us;
        alarm_config.flags.auto_reload_on_alarm = false;

        gptimer_set_alarm_action(shared_timeout_timer, &alarm_config);
        gptimer_set_raw_count(shared_timeout_timer, 0);
        gptimer_start(shared_timeout_timer);

        // Wait for measurement done
        if (xSemaphoreTake(measurement_done_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
            readings[i].distance_cm = static_cast<float>(config.pulse_duration) / 58.0f;
            readings[i].timestamp_us = esp_timer_get_time();
            readings[i].valid = (readings[i].distance_cm > SensorCommon::MIN_DISTANCE_CM && readings[i].distance_cm < SensorCommon::MAX_DISTANCE_CM);
            readings[i].status = readings[i].valid ? 0 : 1;
        } else {
            gptimer_stop(shared_timeout_timer);
            config.pulse_duration = config.timeout_us;
            config.state = SENSOR_IDLE;
            readings[i].distance_cm = SensorCommon::MAX_DISTANCE_CM;
            readings[i].valid = false;
            readings[i].status = 2;
            all_success = false;
        }

        if (readings[i].valid && readings[i].distance_cm < config.collision_threshold_cm) {
            obstacle_detected = true;
        }
    }

    active_sensor_index = -1;

    if (obstacle_detected) {
        int stop_signal = 1;
        xQueueSend(motor_stop_queue, &stop_signal, 0);
    }

    return all_success;
}

// ============================================================================
// UltrasonicSensorManager Implementation
// ============================================================================

UltrasonicSensorManager& UltrasonicSensorManager::instance() {
    static UltrasonicSensorManager instance;
    return instance;
}

UltrasonicSensorManager::UltrasonicSensorManager()
    : array_(nullptr)
    , data_mutex_(nullptr)
    , task_handle_(nullptr)
    , running_(false)
    , paused_(false)
{
    data_mutex_ = xSemaphoreCreateMutex();
}

UltrasonicSensorManager::~UltrasonicSensorManager() {
    stop();
    if (array_) delete array_;
    if (data_mutex_) vSemaphoreDelete(data_mutex_);
}

bool UltrasonicSensorManager::configure(const UltrasonicSensorConfig* sensor_configs,
                                        uint8_t num_sensors) {
    if (array_) {
        ESP_LOGW(TAG, "Manager already configured");
        return false;
    }

    array_ = new UltrasonicSensorArray(num_sensors);

    for (uint8_t i = 0; i < num_sensors; i++) {
        if (!array_->add_sensor(i, sensor_configs[i])) {
            ESP_LOGE(TAG, "Failed to add sensor %d", i);
            delete array_;
            array_ = nullptr;
            return false;
        }
    }

    if (!array_->init()) {
        ESP_LOGE(TAG, "Failed to initialize sensor array");
        delete array_;
        array_ = nullptr;
        return false;
    }

    latest_readings_.resize(num_sensors);
    ESP_LOGI(TAG, "Manager configured with %d sensors", num_sensors);
    return true;
}

bool UltrasonicSensorManager::start_reading_task(uint32_t read_interval_ms, UBaseType_t priority) {
    if (!array_ || task_handle_) {
        return false;
    }

    running_ = true;
    paused_ = false;

    TaskParams* params = new TaskParams{this, read_interval_ms};

    xTaskCreate(reading_task, "ultrasonic_reading", 4096, params, priority, &task_handle_);
    ESP_LOGI(TAG, "Reading task started");
    return true;
}

void UltrasonicSensorManager::reading_task(void* param) {
    auto* params = static_cast<TaskParams*>(param);
    UltrasonicSensorManager* self = params->manager;
    uint32_t interval_ms = params->interval_ms;
    delete params;

    std::vector<SensorCommon::Reading> readings(self->latest_readings_.size());

    while (self->running_) {
        if (!self->paused_) {
            if (self->array_->read_all_single(readings)) {
                if (xSemaphoreTake(self->data_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
                    self->latest_readings_ = readings;
                    xSemaphoreGive(self->data_mutex_);
                }

                if (self->callback_) {
                    self->callback_(readings);
                }

                // Legacy queue
                float distances[NUM_ULTRASONIC_SENSORS];
                size_t size = std::min(readings.size(), static_cast<size_t>(NUM_ULTRASONIC_SENSORS));
                for (size_t j = 0; j < size; ++j) {
                    distances[j] = readings[j].distance_cm;
                }
                xQueueOverwrite(distance_data_queue, distances);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }

    vTaskDelete(NULL);
}

bool UltrasonicSensorManager::get_latest_readings(std::vector<SensorCommon::Reading>& readings) {
    if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        readings = latest_readings_;
        xSemaphoreGive(data_mutex_);
        return true;
    }

    return false;
}

void UltrasonicSensorManager::pause() {
    paused_ = true;
    ESP_LOGI(TAG, "Reading paused");
}

void UltrasonicSensorManager::resume() {
    paused_ = false;
    ESP_LOGI(TAG, "Reading resumed");
}

void UltrasonicSensorManager::stop() {
    running_ = false;
    if (task_handle_) {
        vTaskDelay(pdMS_TO_TICKS(200));
        task_handle_ = nullptr;
    }
    ESP_LOGI(TAG, "Reading stopped");
}

// Legacy functions
void sensor_control_init() {
    distance_data_queue = xQueueCreate(1, sizeof(float) * NUM_ULTRASONIC_SENSORS);
    motor_stop_queue = xQueueCreate(1, sizeof(int));
}

void sensor_control_start_task() {
    UltrasonicSensorManager::instance().start_reading_task(100, 5);
}