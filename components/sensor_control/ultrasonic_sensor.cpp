// Complete ultrasonic_sensor.cpp
#include "ultrasonic_sensor.hpp"

static const char* TAG = "Ultrasonic_Sensor";

// Queue definition - distance_data_queue only
QueueHandle_t distance_data_queue = nullptr;

// Shared resources for ISRs
static gptimer_handle_t shared_timeout_timer = NULL;
static SemaphoreHandle_t measurement_done_sem = NULL;
static volatile int active_sensor_index = -1;
static UltrasonicSensorArray* g_array = nullptr;  // Global for ISRs to access configs

// ============================================================================
// ISR Handlers
// ============================================================================

static void IRAM_ATTR echo_isr_handler(void* arg) {
    int sensor_idx = (int)(intptr_t)arg;
    if (sensor_idx < 0 || sensor_idx >= SensorCommon::NUM_ULTRASONIC_SENSORS || !g_array) return;

    UltrasonicSensorConfig& cfg = g_array->configs_[sensor_idx];
    uint32_t gpio_level = gpio_get_level(cfg.echo_pin);

    if (gpio_level == 1) {  // Rising edge - start of pulse
        cfg.start_time = esp_timer_get_time();
        cfg.state = SENSOR_MEASURING;
    } else if (gpio_level == 0 && cfg.state == SENSOR_MEASURING) {  // Falling edge - end of pulse
        cfg.pulse_duration = (uint32_t)(esp_timer_get_time() - cfg.start_time);
        cfg.state = SENSOR_IDLE;
        xSemaphoreGiveFromISR(measurement_done_sem, NULL);
    }
}

static bool IRAM_ATTR timeout_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_data) {
    if (active_sensor_index >= 0 && active_sensor_index < SensorCommon::NUM_ULTRASONIC_SENSORS && g_array) {
        UltrasonicSensorConfig& cfg = g_array->configs_[active_sensor_index];
        cfg.pulse_duration = cfg.timeout_us;  // Indicate timeout
        cfg.state = SENSOR_IDLE;
        xSemaphoreGiveFromISR(measurement_done_sem, NULL);
    }
    active_sensor_index = -1;
    return false;  // Don't need to yield
}

// ============================================================================
// UltrasonicSensorArray Implementation
// ============================================================================

UltrasonicSensorArray::UltrasonicSensorArray(uint8_t num_sensors)
    : num_sensors_(num_sensors), initialized_(false) {
    ESP_LOGI(TAG, "Creating UltrasonicSensorArray with %d sensors", num_sensors);
    array_mutex_ = xSemaphoreCreateMutex();
    if (!array_mutex_) {
        ESP_LOGE(TAG, "Failed to create array mutex");
    }
    memset(configs_, 0, sizeof(configs_));
}

UltrasonicSensorArray::~UltrasonicSensorArray() {
    ESP_LOGI(TAG, "Destroying UltrasonicSensorArray");

    if (shared_timeout_timer) {
        gptimer_stop(shared_timeout_timer);
        gptimer_disable(shared_timeout_timer);
        gptimer_del_timer(shared_timeout_timer);
        shared_timeout_timer = NULL;
    }

    if (measurement_done_sem) {
        vSemaphoreDelete(measurement_done_sem);
        measurement_done_sem = NULL;
    }

    if (array_mutex_) {
        vSemaphoreDelete(array_mutex_);
        array_mutex_ = NULL;
    }

    g_array = nullptr;
    active_sensor_index = -1;
}

bool UltrasonicSensorArray::init() {
    if (initialized_) {
        ESP_LOGW(TAG, "Array already initialized");
        return true;
    }

    // Create semaphore
    measurement_done_sem = xSemaphoreCreateBinary();
    if (!measurement_done_sem) {
        ESP_LOGE(TAG, "Failed to create measurement semaphore");
        return false;
    }

    // Configure timeout timer - with all fields initialized
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 MHz, 1us per tick
        .intr_priority = 0,
        .flags = {
            .intr_shared = false,
            .backup_before_sleep = false
        }
    };

    esp_err_t ret = gptimer_new_timer(&timer_config, &shared_timeout_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timeout timer: %s", esp_err_to_name(ret));
        vSemaphoreDelete(measurement_done_sem);
        measurement_done_sem = NULL;
        return false;
    }

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timeout_timer_isr,
    };
    gptimer_register_event_callbacks(shared_timeout_timer, &cbs, NULL);

    ret = gptimer_enable(shared_timeout_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable timeout timer: %s", esp_err_to_name(ret));
        gptimer_del_timer(shared_timeout_timer);
        shared_timeout_timer = NULL;
        vSemaphoreDelete(measurement_done_sem);
        measurement_done_sem = NULL;
        return false;
    }

    g_array = this;
    initialized_ = true;
    ESP_LOGI(TAG, "UltrasonicSensorArray initialized");
    return true;
}

bool UltrasonicSensorArray::add_sensor(uint8_t index, const UltrasonicSensorConfig& sensor_config) {
    if (index >= num_sensors_) {
        ESP_LOGE(TAG, "Sensor index %d out of bounds (max %d)", index, num_sensors_);
        return false;
    }

    if (xSemaphoreTake(array_mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take array mutex for sensor %d", index);
        return false;
    }

    // Copy configuration
    configs_[index] = sensor_config;

    // Configure GPIOs
    gpio_config_t trig_config = {
        .pin_bit_mask = (1ULL << sensor_config.trig_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&trig_config);
    gpio_set_level(sensor_config.trig_pin, 0);

    gpio_config_t echo_config = {
        .pin_bit_mask = (1ULL << sensor_config.echo_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,  // Pull down to avoid floating
        .intr_type = GPIO_INTR_ANYEDGE,  // Trigger on both edges
    };
    gpio_config(&echo_config);

    // Install ISR service if not already installed
    static bool isr_service_installed = false;
    if (!isr_service_installed) {
        gpio_install_isr_service(0);
        isr_service_installed = true;
    }

    // Attach ISR handler
    gpio_isr_handler_add(sensor_config.echo_pin, echo_isr_handler, (void*)(intptr_t)index);

    ESP_LOGI(TAG, "Added ultrasonic sensor %d (TRIG: GPIO%d, ECHO: GPIO%d)",
             index, sensor_config.trig_pin, sensor_config.echo_pin);

    xSemaphoreGive(array_mutex_);
    return true;
}

bool UltrasonicSensorArray::read_all_single(std::vector<SensorCommon::Reading>& readings, uint32_t timeout_ms) {
    if (!initialized_ || num_sensors_ == 0) {
        ESP_LOGE(TAG, "Array not initialized or no sensors");
        return false;
    }

    if (xSemaphoreTake(array_mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take array mutex for reading");
        return false;
    }

    readings.resize(num_sensors_);
    bool all_success = true;

    for (int i = 0; i < num_sensors_; i++) {
        UltrasonicSensorConfig& cfg = configs_[i];

        // Reset state
        cfg.state = SENSOR_IDLE;
        cfg.pulse_duration = 0;
        cfg.start_time = 0;

        // Disable interrupts during trigger
        gpio_intr_disable(cfg.echo_pin);

        // Send 10us trigger pulse
        gpio_set_level(cfg.trig_pin, 1);
        esp_rom_delay_us(10);
        gpio_set_level(cfg.trig_pin, 0);

        // Re-enable interrupts
        gpio_intr_enable(cfg.echo_pin);

        // Set active sensor for timeout
        active_sensor_index = i;

        // Configure and start timeout timer
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = cfg.timeout_us,
            .reload_count = 0,
            .flags = { .auto_reload_on_alarm = false }
        };
        gptimer_set_alarm_action(shared_timeout_timer, &alarm_config);
        gptimer_start(shared_timeout_timer);

        // Wait for measurement to complete (either echo received or timeout)
        if (xSemaphoreTake(measurement_done_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
            // Stop timer
            gptimer_stop(shared_timeout_timer);

            // Calculate distance (speed of sound = 34300 cm/s = 0.0343 cm/us)
            // Distance = (time * speed_of_sound) / 2 (round trip)
            float distance_cm = (cfg.pulse_duration * 0.0343f) / 2.0f;

            readings[i].distance_cm = distance_cm;
            readings[i].valid = (distance_cm >= SensorCommon::MIN_DISTANCE_CM &&
                               distance_cm <= SensorCommon::MAX_DISTANCE_CM);
            readings[i].status = readings[i].valid ? 0 :
                                (cfg.pulse_duration == cfg.timeout_us ? 1 : 2); // 1=timeout, 2=invalid range
            readings[i].timestamp_us = esp_timer_get_time();

            if (!readings[i].valid && cfg.pulse_duration != cfg.timeout_us) {
                ESP_LOGW(TAG, "Sensor %d: invalid reading %.1f cm", i, distance_cm);
            }
        } else {
            ESP_LOGW(TAG, "Sensor %d: measurement timeout", i);
            gptimer_stop(shared_timeout_timer);
            readings[i].distance_cm = SensorCommon::MAX_DISTANCE_CM;
            readings[i].valid = false;
            readings[i].status = 1; // Timeout
            readings[i].timestamp_us = esp_timer_get_time();
            all_success = false;
        }

        // Small delay between sensors
        if (i < num_sensors_ - 1) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    active_sensor_index = -1;
    xSemaphoreGive(array_mutex_);

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
    , paused_(false) {
    data_mutex_ = xSemaphoreCreateMutex();
    if (!data_mutex_) {
        ESP_LOGE(TAG, "Failed to create data mutex");
    }

    // Create the distance_data_queue if not already created
    if (!distance_data_queue) {
        distance_data_queue = xQueueCreate(1, sizeof(float) * SensorCommon::NUM_ULTRASONIC_SENSORS);
        if (!distance_data_queue) {
            ESP_LOGE(TAG, "Failed to create distance_data_queue");
        }
    }
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
  if (task_handle_) {
    return false;
  }

  // Dynamically allocate TaskParams to avoid stack invalidation
  auto* params = new TaskParams{this, read_interval_ms};
  if (!params) {
    ESP_LOGE(TAG, "Failed to allocate TaskParams");
    return false;
  }

  running_ = true;
  paused_ = false;

  BaseType_t res = xTaskCreate(reading_task, "usonic_read", 4096, params, priority, &task_handle_);
  if (res != pdPASS) {
    delete params;  // Clean up on failure
    running_ = false;
    return false;
  }

  ESP_LOGI(TAG, "Reading task started with interval %lu ms", read_interval_ms);
  return true;
}

void UltrasonicSensorManager::reading_task(void* param) {
  TaskParams* params = static_cast<TaskParams*>(param);
  UltrasonicSensorManager* self = params->manager;
  uint32_t interval_ms = params->interval_ms;

  ESP_LOGI(TAG, "Ultrasonic reading task running");

  std::vector<SensorCommon::Reading> readings(self->latest_readings_.size());

  while (self->running_) {
    if (!self->paused_) {
      if (self->array_->read_all_single(readings, interval_ms)) {
        if (xSemaphoreTake(self->data_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
          self->latest_readings_ = readings;
          xSemaphoreGive(self->data_mutex_);
        }

        if (self->callback_) {
          self->callback_(readings);
        }

        // Legacy queue
        if (distance_data_queue) {
          float distances[SensorCommon::NUM_ULTRASONIC_SENSORS];
          size_t size = std::min(readings.size(), static_cast<size_t>(SensorCommon::NUM_ULTRASONIC_SENSORS));
          for (size_t j = 0; j < size; ++j) {
            distances[j] = readings[j].distance_cm;
          }
          xQueueOverwrite(distance_data_queue, distances);
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(interval_ms));
  }

  ESP_LOGI(TAG, "Ultrasonic reading task exiting");

  // Clean up the dynamically allocated params
  delete params;

  vTaskDelete(nullptr);
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
