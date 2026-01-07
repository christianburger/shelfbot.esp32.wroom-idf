// Updated tof_sensor.cpp with interrupt support
#include "tof_sensor.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include <cstring>
#include <vector>

static const char* TAG = "ToF_Sensor";

// VL53L0X Register Definitions
namespace VL53L0X_Reg {
    constexpr uint8_t IDENTIFICATION_MODEL_ID = 0xC0;
    constexpr uint8_t IDENTIFICATION_REVISION_ID = 0xC2;
    constexpr uint8_t SYSRANGE_START = 0x00;
    constexpr uint8_t RESULT_INTERRUPT_STATUS = 0x13;
    constexpr uint8_t RESULT_RANGE_STATUS = 0x14;
    constexpr uint8_t I2C_SLAVE_DEVICE_ADDRESS = 0x8A;
    constexpr uint8_t SYSTEM_INTERMEASUREMENT_PERIOD = 0x04;
    constexpr uint8_t GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0;
    constexpr uint8_t MSRC_CONFIG_CONTROL = 0x60;
    constexpr uint8_t FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44;
    constexpr uint8_t SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A;
    constexpr uint8_t GPIO_HV_MUX_ACTIVE_HIGH = 0x84;
    constexpr uint8_t SYSTEM_INTERRUPT_CLEAR_GPIO = 0x0B;
}

// ============================================================================
// ToFSensor Implementation
// ============================================================================

ToFSensor::ToFSensor(const ToFSensorConfig& config)
    : config_(config)
    , initialized_(false)
    , continuous_mode_(false)
    , i2c_mutex_(nullptr)
    , measurement_sem_(nullptr)
{
    i2c_mutex_ = xSemaphoreCreateMutex();
    measurement_sem_ = xSemaphoreCreateBinary();
}

ToFSensor::~ToFSensor() {
    if (i2c_mutex_) {
        vSemaphoreDelete(i2c_mutex_);
    }
    if (measurement_sem_) {
        vSemaphoreDelete(measurement_sem_);
    }
}

bool ToFSensor::init() {
    if (initialized_) {
        return true;
    }

    // If XSHUT pin is configured, assert it to enable sensor
    if (config_.xshut_gpio != 255) {
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << config_.xshut_gpio);
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&io_conf);

        gpio_set_level((gpio_num_t)config_.xshut_gpio, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level((gpio_num_t)config_.xshut_gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Verify sensor model
    uint8_t model_id = 0;
    if (!read_register(VL53L0X_Reg::IDENTIFICATION_MODEL_ID, model_id)) {
        ESP_LOGE(TAG, "Failed to read model ID from sensor at 0x%02X", config_.i2c_address);
        return false;
    }

    if (model_id != 0xEE) {
        ESP_LOGE(TAG, "Unexpected model ID: 0x%02X (expected 0xEE)", model_id);
        return false;
    }

    ESP_LOGI(TAG, "VL53L0X detected at address 0x%02X", config_.i2c_address);

    // Basic initialization sequence for VL53L0X
    // This is a simplified version - full init is more complex
    write_register(0x88, 0x00);
    write_register(0x80, 0x01);
    write_register(0xFF, 0x01);
    write_register(0x00, 0x00);
    uint8_t stop_variable;
    read_register(0x91, stop_variable);
    write_register(0x00, 0x01);
    write_register(0xFF, 0x00);
    write_register(0x80, 0x00);

    // Set signal rate limit
    if (!set_signal_rate_limit(0.25f)) {
        ESP_LOGW(TAG, "Failed to set signal rate limit");
    }

    // Configure SPAD
    uint8_t spad_count;
    bool spad_type_is_aperture;
    if (!get_spad_info(spad_count, spad_type_is_aperture)) {
        ESP_LOGW(TAG, "Failed to get SPAD info");
    }

    // Set measurement timing budget
    if (!set_timing_budget(config_.timing_budget_ms)) {
        ESP_LOGW(TAG, "Failed to set timing budget");
    }

    // Configure interrupt if int_gpio is set
    if (config_.int_gpio != GPIO_NUM_MAX) {
        if (!configure_interrupt()) {
            ESP_LOGW(TAG, "Failed to configure interrupt");
        }
    }

    initialized_ = true;
    ESP_LOGI(TAG, "Sensor at 0x%02X initialized successfully", config_.i2c_address);
    return true;
}

bool ToFSensor::configure_interrupt() {
    // Set interrupt config to new sample ready (0x04)
    if (!write_register(VL53L0X_Reg::SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)) {
        return false;
    }

    // Set polarity: bit 4 in GPIO_HV_MUX_ACTIVE_HIGH (0 = active low, 1 = active high)
    uint8_t polarity = config_.interrupt_active_high ? 0x10 : 0x00;
    if (!write_register(VL53L0X_Reg::GPIO_HV_MUX_ACTIVE_HIGH, polarity)) {
        return false;
    }

    // Configure ESP32 GPIO for interrupt
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << config_.int_gpio);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;  // Since open-drain
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = config_.interrupt_active_high ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE;
    if (gpio_config(&io_conf) != ESP_OK) {
        return false;
    }

    // Install ISR service if not already (call once per app)
    gpio_install_isr_service(0);

    // Add handler
    if (gpio_isr_handler_add(config_.int_gpio, interrupt_handler, this) != ESP_OK) {
        return false;
    }

    return true;
}

void IRAM_ATTR ToFSensor::interrupt_handler(void* arg) {
    ToFSensor* self = static_cast<ToFSensor*>(arg);
    BaseType_t task_woken = pdFALSE;
    xSemaphoreGiveFromISR(self->measurement_sem_, &task_woken);
    if (task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

bool ToFSensor::read_single(Reading& reading, uint32_t timeout_ms) {
    if (!initialized_) {
        return false;
    }

    if (xSemaphoreTake(i2c_mutex_, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return false;
    }

    // Start single measurement
    write_register(0x80, 0x01);
    write_register(0xFF, 0x01);
    write_register(0x00, 0x00);
    write_register(0x91, 0x3C);  // stop_variable
    write_register(0x00, 0x01);
    write_register(0xFF, 0x00);
    write_register(0x80, 0x00);
    write_register(VL53L0X_Reg::SYSRANGE_START, 0x01);

    bool use_interrupt = (config_.int_gpio != GPIO_NUM_MAX);

    if (use_interrupt) {
        // Wait for interrupt semaphore
        if (xSemaphoreTake(measurement_sem_, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
            ESP_LOGW(TAG, "Interrupt timeout");
            xSemaphoreGive(i2c_mutex_);
            return false;
        }
    } else {
        // Poll as before
        uint32_t start_time = esp_timer_get_time() / 1000;
        uint8_t status;
        do {
            if (!read_register(VL53L0X_Reg::RESULT_INTERRUPT_STATUS, status)) {
                xSemaphoreGive(i2c_mutex_);
                return false;
            }

            if ((esp_timer_get_time() / 1000 - start_time) > timeout_ms) {
                ESP_LOGW(TAG, "Measurement timeout");
                xSemaphoreGive(i2c_mutex_);
                return false;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        } while ((status & 0x07) == 0);
    }

    // Read distance
    uint16_t distance_mm;
    if (!read_register_16(VL53L0X_Reg::RESULT_RANGE_STATUS + 10, distance_mm)) {
        xSemaphoreGive(i2c_mutex_);
        return false;
    }

    // Read status
    uint8_t range_status;
    read_register(VL53L0X_Reg::RESULT_RANGE_STATUS, range_status);

    // Clear interrupt
    write_register(VL53L0X_Reg::SYSTEM_INTERRUPT_CLEAR_GPIO, 0x01);

    reading.distance_cm = distance_mm / 10.0f;  // Convert mm to cm for unified unit
    reading.status = (range_status >> 3) & 0x0F;
    reading.timestamp_us = esp_timer_get_time();
    reading.valid = (reading.status == 0);

    xSemaphoreGive(i2c_mutex_);
    return true;
}

bool ToFSensor::start_continuous() {
    if (!initialized_) {
        return false;
    }

    // Set continuous mode with inter-measurement period
    write_register(0x80, 0x01);
    write_register(0xFF, 0x01);
    write_register(0x00, 0x00);
    write_register(0x91, 0x3C);
    write_register(0x00, 0x01);
    write_register(0xFF, 0x00);
    write_register(0x80, 0x00);
    write_register(VL53L0X_Reg::SYSRANGE_START, 0x02);

    continuous_mode_ = true;
    return true;
}

bool ToFSensor::stop_continuous() {
    write_register(VL53L0X_Reg::SYSRANGE_START, 0x01);
    continuous_mode_ = false;
    return true;
}

bool ToFSensor::set_address(uint8_t new_address) {
    if (new_address < ToFConstants::ADDRESS_MIN || new_address > ToFConstants::ADDRESS_MAX) {
        return false;
    }

    if (write_register(VL53L0X_Reg::I2C_SLAVE_DEVICE_ADDRESS, new_address & 0x7F)) {
        config_.i2c_address = new_address;
        return true;
    }
    return false;
}

uint8_t ToFSensor::get_address() const {
    return config_.i2c_address;
}

// ============================================================================
// Private I2C Methods
// ============================================================================

bool ToFSensor::write_register(uint8_t reg, uint8_t value) {
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_write_to_device(config_.i2c_port, config_.i2c_address,
                                      write_buf, 2, pdMS_TO_TICKS(100)) == ESP_OK;
}

bool ToFSensor::write_register_16(uint8_t reg, uint16_t value) {
    uint8_t write_buf[3] = {reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    return i2c_master_write_to_device(config_.i2c_port, config_.i2c_address,
                                      write_buf, 3, pdMS_TO_TICKS(100)) == ESP_OK;
}

bool ToFSensor::read_register(uint8_t reg, uint8_t& value) {
    return i2c_master_write_read_device(config_.i2c_port, config_.i2c_address,
                                        &reg, 1, &value, 1, pdMS_TO_TICKS(100)) == ESP_OK;
}

bool ToFSensor::read_register_16(uint8_t reg, uint16_t& value) {
    uint8_t data[2];
    if (i2c_master_write_read_device(config_.i2c_port, config_.i2c_address,
                                     &reg, 1, data, 2, pdMS_TO_TICKS(100)) == ESP_OK) {
        value = ((uint16_t)data[0] << 8) | data[1];
        return true;
    }
    return false;
}

bool ToFSensor::set_signal_rate_limit(float limit_mcps) {
    uint16_t value = (uint16_t)(limit_mcps * 128);
    return write_register_16(VL53L0X_Reg::FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, value);
}

bool ToFSensor::get_spad_info(uint8_t& count, bool& type_is_aperture) {
    // Simplified SPAD calibration
    write_register(0x80, 0x01);
    write_register(0xFF, 0x01);
    write_register(0x00, 0x00);
    write_register(0xFF, 0x06);

    uint8_t tmp;
    read_register(0x83, tmp);
    write_register(0x83, tmp | 0x04);
    write_register(0xFF, 0x07);
    write_register(0x81, 0x01);
    write_register(0x80, 0x01);
    write_register(0x94, 0x6b);
    write_register(0x83, 0x00);

    // Wait for ready
    vTaskDelay(pdMS_TO_TICKS(10));

    write_register(0x83, 0x01);
    uint8_t spad_map[6];
    read_register(0x92, spad_map[0]);

    write_register(0x81, 0x00);
    write_register(0xFF, 0x06);
    read_register(0x83, tmp);
    write_register(0x83, tmp & ~0x04);
    write_register(0xFF, 0x01);
    write_register(0x00, 0x01);
    write_register(0xFF, 0x00);
    write_register(0x80, 0x00);

    count = spad_map[0];
    type_is_aperture = false;
    return true;
}

bool ToFSensor::set_timing_budget(uint16_t budget_ms) {
    // Simplified timing budget setting
    if (budget_ms < ToFConstants::MIN_TIMING_BUDGET_MS ||
        budget_ms > ToFConstants::MAX_TIMING_BUDGET_MS) {
        return false;
    }
    config_.timing_budget_ms = budget_ms;
    return true;
}

// ============================================================================
// ToFSensorArray Implementation
// ============================================================================

ToFSensorArray::ToFSensorArray(const ToFArrayConfig& config)
    : config_(config)
    , sensors_(nullptr)
    , sensor_count_(0)
    , initialized_(false)
    , array_mutex_(nullptr)
{
    array_mutex_ = xSemaphoreCreateMutex();
    sensors_ = new ToFSensor*[config_.num_sensors];
    memset(sensors_, 0, sizeof(ToFSensor*) * config_.num_sensors);
}

ToFSensorArray::~ToFSensorArray() {
    if (sensors_) {
        for (uint8_t i = 0; i < sensor_count_; i++) {
            delete sensors_[i];
        }
        delete[] sensors_;
    }
    if (array_mutex_) {
        vSemaphoreDelete(array_mutex_);
    }
}

bool ToFSensorArray::init() {
    if (!init_i2c_bus()) {
        return false;
    }

    if (!program_sensor_addresses()) {
        return false;
    }

    initialized_ = true;
    return true;
}

bool ToFSensorArray::init_i2c_bus() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = config_.sda_gpio;
    conf.scl_io_num = config_.scl_gpio;
    conf.sda_pullup_en = config_.enable_pullups ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    conf.scl_pullup_en = config_.enable_pullups ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = config_.i2c_freq_hz;
    conf.clk_flags = 0;

    esp_err_t err = i2c_param_config(config_.i2c_port, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return false;
    }

    err = i2c_driver_install(config_.i2c_port, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "I2C bus initialized on port %d", config_.i2c_port);
    return true;
}

bool ToFSensorArray::program_sensor_addresses() {
    // All sensors start with default address, so we must enable them
    // one at a time and reprogram their addresses
    ESP_LOGI(TAG, "Programming %d sensor addresses", sensor_count_);

    for (uint8_t i = 0; i < sensor_count_; i++) {
        if (!sensors_[i]->init()) {
            ESP_LOGE(TAG, "Failed to initialize sensor %d", i);
            return false;
        }

        // If this is not the last sensor and it still has default address, change it
        if (i < sensor_count_ - 1 &&
            sensors_[i]->get_address() == ToFConstants::DEFAULT_ADDRESS) {
            uint8_t new_addr = ToFConstants::DEFAULT_ADDRESS + i + 1;
            if (!sensors_[i]->set_address(new_addr)) {
                ESP_LOGE(TAG, "Failed to set address for sensor %d", i);
                return false;
            }
            ESP_LOGI(TAG, "Sensor %d address changed to 0x%02X", i, new_addr);
        }
    }

    return true;
}

bool ToFSensorArray::add_sensor(const ToFSensorConfig& sensor_config) {
    if (sensor_count_ >= config_.num_sensors) {
        ESP_LOGE(TAG, "Cannot add more sensors, array full");
        return false;
    }

    sensors_[sensor_count_] = new ToFSensor(sensor_config);
    sensor_count_++;
    return true;
}

bool ToFSensorArray::read_all_single(std::vector<Reading>& readings, uint32_t timeout_ms) {
    if (!initialized_) {
        return false;
    }

    readings.resize(sensor_count_);
    bool all_success = true;
    for (uint8_t i = 0; i < sensor_count_; i++) {
        if (!sensors_[i]->read_single(readings[i], timeout_ms)) {
            readings[i].valid = false;
            all_success = false;
        }
    }

    return all_success;
}

// ============================================================================
// ToFSensorManager Implementation
// ============================================================================

ToFSensorManager& ToFSensorManager::instance() {
    static ToFSensorManager instance;
    return instance;
}

ToFSensorManager::ToFSensorManager()
    : array_(nullptr)
    , data_mutex_(nullptr)
    , task_handle_(nullptr)
    , running_(false)
    , paused_(false)
{
    data_mutex_ = xSemaphoreCreateMutex();
}

ToFSensorManager::~ToFSensorManager() {
    stop();
    if (array_) delete array_;
    if (data_mutex_) vSemaphoreDelete(data_mutex_);
}

bool ToFSensorManager::configure(const ToFArrayConfig& array_config,
                                 const ToFSensorConfig* sensor_configs,
                                 uint8_t num_sensors) {
    if (array_) {
        ESP_LOGW(TAG, "Manager already configured");
        return false;
    }

    array_ = new ToFSensorArray(array_config);

    for (uint8_t i = 0; i < num_sensors; i++) {
        array_->add_sensor(sensor_configs[i]);
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

bool ToFSensorManager::start_reading_task(uint32_t read_interval_ms, UBaseType_t priority) {
    if (!array_ || task_handle_) {
        return false;
    }

    running_ = true;
    paused_ = false;

    TaskParams* params = new TaskParams{this, read_interval_ms};

    xTaskCreate(reading_task, "tof_reading", 4096, params, priority, &task_handle_);
    ESP_LOGI(TAG, "Reading task started");
    return true;
}

void ToFSensorManager::reading_task(void* param) {
    auto* params = static_cast<TaskParams*>(param);
    ToFSensorManager* self = params->manager;
    uint32_t interval_ms = params->interval_ms;
    delete params;

    std::vector<Reading> readings(self->latest_readings_.size());

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
            }
        }

        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }

    vTaskDelete(nullptr);
}

bool ToFSensorManager::get_latest_readings(std::vector<Reading>& readings) {
    if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        readings = latest_readings_;  // Copy vector
        xSemaphoreGive(data_mutex_);
        return true;
    }

    return false;
}

void ToFSensorManager::pause() {
    paused_ = true;
    ESP_LOGI(TAG, "Reading paused");
}

void ToFSensorManager::resume() {
    paused_ = false;
    ESP_LOGI(TAG, "Reading resumed");
}

void ToFSensorManager::stop() {
    running_ = false;
    if (task_handle_) {
        vTaskDelay(pdMS_TO_TICKS(200)); // Allow task to finish
        task_handle_ = nullptr;
    }
    ESP_LOGI(TAG, "Reading stopped");
}