#include <idf_c_includes.hpp>
#include "sensor_control.hpp"
#include "ultrasonic_sensor.hpp"
#include "vl53l1_modbus.hpp"
#include "sensor_common.hpp"

const char* SensorControl::TAG = "SensorControl";

// Global instance for C-style interface
static SensorControl* g_sensor_control = nullptr;
static SensorCommon::SensorDataPacket g_latest_data = {};
static SemaphoreHandle_t g_data_mutex = nullptr;

// Simple TOF sensor implementation using VL53L1_Modbus
class TofSensorImpl {
private:
    std::unique_ptr<VL53L1_Modbus> driver_;
    VL53L1_Modbus::Config config_;
    static constexpr const char* TAG = "TofSensorImpl";

public:
    TofSensorImpl() {
        config_ = vl53l1_modbus_default_config(
            I2C_NUM_0,
            GPIO_NUM_21,
            GPIO_NUM_22,
            UART_NUM_1,
            GPIO_NUM_17,
            GPIO_NUM_16
        );

        driver_ = std::make_unique<VL53L1_Modbus>(config_);
    }

    esp_err_t initialize() {
        const char* err = driver_->init();
        if (err != nullptr) {
            ESP_LOGE(TAG, "TOF sensor init failed: %s", err);
            return ESP_FAIL;
        }
        return ESP_OK;
    }

    bool is_ready() const {
        return driver_ && driver_->isReady();
    }

    esp_err_t read_measurement(uint16_t& distance_mm, bool& valid, uint8_t& status) {
        if (!driver_) {
            return ESP_ERR_INVALID_STATE;
        }

        VL53L1_Modbus::Measurement result;
        if (!driver_->readContinuous(result)) {
            return ESP_ERR_INVALID_RESPONSE;
        }

        distance_mm = result.distance_mm;
        valid = result.valid;
        status = result.range_status;
        return ESP_OK;
    }

    esp_err_t set_mode(bool long_distance) {
        if (!driver_) {
            return ESP_ERR_INVALID_STATE;
        }

        VL53L1_Modbus::RangingMode mode = long_distance
            ? VL53L1_Modbus::RangingMode::LONG_DISTANCE
            : VL53L1_Modbus::RangingMode::HIGH_PRECISION;

        const char* err = driver_->setRangingMode(mode);
        return (err == nullptr) ? ESP_OK : ESP_FAIL;
    }

    esp_err_t start_continuous() {
        if (!driver_) {
            return ESP_ERR_INVALID_STATE;
        }

        return driver_->startContinuous() ? ESP_OK : ESP_FAIL;
    }

    esp_err_t stop_continuous() {
        if (!driver_) {
            return ESP_ERR_INVALID_STATE;
        }

        return driver_->stopContinuous() ? ESP_OK : ESP_FAIL;
    }

    bool probe() {
        return driver_ && driver_->probe();
    }
};

// Constructor - Fixed initialization order to match member declaration order
SensorControl::SensorControl(const Config& config)
    : config_(config),
      ultrasonic_array_(nullptr),
      tof_sensor_(nullptr),
      initialized_(false),
      continuous_mode_(false),
      continuous_task_handle_(nullptr) {}

SensorControl::~SensorControl() {
    stop_continuous();

    if (continuous_task_handle_) {
        vTaskDelete(continuous_task_handle_);
    }

    if (tof_sensor_) {
        delete static_cast<TofSensorImpl*>(tof_sensor_);
    }
}

esp_err_t SensorControl::initialize_ultrasonic() {
    if (config_.ultrasonic_configs.empty()) {
        ESP_LOGW(TAG, "No ultrasonic sensors configured");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing %zu ultrasonic sensor(s)...",
             config_.ultrasonic_configs.size());

    ultrasonic_array_ = std::make_unique<UltrasonicSensorArray>();

    for (size_t i = 0; i < config_.ultrasonic_configs.size(); i++) {
        const auto& uconfig = config_.ultrasonic_configs[i];

        // Use UltrasonicSensorConfig from ultrasonic_sensor.hpp
        UltrasonicSensorConfig sensor_config = {
            .trig_pin = static_cast<gpio_num_t>(uconfig.trig_pin),
            .echo_pin = static_cast<gpio_num_t>(uconfig.echo_pin),
            .collision_threshold_cm = 20.0f,  // Default threshold
            .timeout_us = uconfig.timeout_us,
            .state = SENSOR_IDLE,
            .start_time = 0,
            .pulse_duration = 0
        };

        // Add sensor to the array
        if (!ultrasonic_array_->add_sensor(i, sensor_config)) {
            ESP_LOGE(TAG, "Failed to add ultrasonic sensor %zu", i);
            return ESP_FAIL;
        }

        ESP_LOGD(TAG, "Added ultrasonic sensor %zu: TRIG=GPIO%d, ECHO=GPIO%d",
                i, uconfig.trig_pin, uconfig.echo_pin);
    }

    // Initialize the array
    if (!ultrasonic_array_->init()) {
        ESP_LOGE(TAG, "Failed to initialize ultrasonic sensors");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Ultrasonic sensors initialized successfully");
    return ESP_OK;
}

esp_err_t SensorControl::initialize_tof() {
    ESP_LOGI(TAG, "Initializing TOF sensor...");

    tof_sensor_ = new TofSensorImpl();
    esp_err_t err = static_cast<TofSensorImpl*>(tof_sensor_)->initialize();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TOF sensor initialization failed");
        delete static_cast<TofSensorImpl*>(tof_sensor_);
        tof_sensor_ = nullptr;
        return err;
    }

    ESP_LOGI(TAG, "TOF sensor initialized successfully");
    return ESP_OK;
}

esp_err_t SensorControl::initialize() {
    if (initialized_) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "Initializing Unified Sensor Control");
    ESP_LOGI(TAG, "=========================================");

    esp_err_t err = initialize_ultrasonic();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ultrasonic initialization failed");
        return err;
    }

    err = initialize_tof();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TOF initialization failed");
        ESP_LOGW(TAG, "Continuing without TOF sensor");
    }

    initialized_ = true;

    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "Sensor Control Initialization Complete");
    ESP_LOGI(TAG, "=========================================");

    return ESP_OK;
}

bool SensorControl::is_ready() const {
    bool ultrasonic_ready = config_.ultrasonic_configs.empty() ||
                           (ultrasonic_array_ != nullptr);

    bool tof_ready = (tof_sensor_ == nullptr) ||
                    (static_cast<TofSensorImpl*>(tof_sensor_)->is_ready());

    return initialized_ && ultrasonic_ready && tof_ready;
}

esp_err_t SensorControl::read_ultrasonic(std::vector<uint16_t>& distances) {
    distances.clear();

    if (!ultrasonic_array_ || config_.ultrasonic_configs.empty()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Read from ultrasonic sensor array
    std::vector<SensorCommon::Reading> readings;
    if (!ultrasonic_array_->read_all_single(readings, SensorCommon::DEFAULT_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "Failed to read ultrasonic sensors");
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Convert readings to distances in mm
    for (const auto& reading : readings) {
        // Convert cm to mm and ensure it's within uint16_t range
        uint16_t distance_mm = static_cast<uint16_t>(reading.distance_cm * 10);
        distances.push_back(distance_mm);
    }

    return ESP_OK;
}

esp_err_t SensorControl::read_tof(uint16_t& distance_mm, bool& valid, uint8_t& status) {
    if (!tof_sensor_) {
        return ESP_ERR_INVALID_STATE;
    }

    return static_cast<TofSensorImpl*>(tof_sensor_)->read_measurement(distance_mm, valid, status);
}

esp_err_t SensorControl::read_all(std::vector<uint16_t>& ultrasonic_distances,
                                 uint16_t& tof_distance_mm, bool& tof_valid, uint8_t& tof_status) {
    esp_err_t err = ESP_OK;

    esp_err_t ultrasonic_err = read_ultrasonic(ultrasonic_distances);
    if (ultrasonic_err != ESP_OK && ultrasonic_array_) {
        err = ultrasonic_err;
    }

    esp_err_t tof_err = read_tof(tof_distance_mm, tof_valid, tof_status);
    if (tof_err != ESP_OK && tof_sensor_) {
        err = tof_err;
    }

    return err;
}

void SensorControl::continuous_read_loop() {
    ESP_LOGI(TAG, "Starting continuous sensor reading...");

    TickType_t last_ultrasonic_wake = xTaskGetTickCount();
    TickType_t last_tof_wake = xTaskGetTickCount();

    while (continuous_mode_) {
        TickType_t now = xTaskGetTickCount();

        // Read all sensors and update global data packet
        if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Read ultrasonic sensors
            std::vector<uint16_t> ultrasonic_distances;
            if (read_ultrasonic(ultrasonic_distances) == ESP_OK) {
                for (size_t i = 0; i < ultrasonic_distances.size() && i < SensorCommon::NUM_ULTRASONIC_SENSORS; i++) {
                    g_latest_data.ultrasonic_distances_cm[i] = ultrasonic_distances[i] / 10.0f;  // mm to cm
                }
            }

            // Read ToF sensor
            uint16_t tof_distance_mm;
            bool tof_valid;
            uint8_t tof_status;
            if (read_tof(tof_distance_mm, tof_valid, tof_status) == ESP_OK) {
                g_latest_data.tof_distances_cm[0] = tof_distance_mm / 10.0f;  // mm to cm
                g_latest_data.tof_valid[0] = tof_valid;
                g_latest_data.tof_status[0] = tof_status;
            }

            g_latest_data.timestamp_us = esp_timer_get_time();

            xSemaphoreGive(g_data_mutex);

            // Call legacy callbacks if set
            if ((now - last_ultrasonic_wake) * portTICK_PERIOD_MS >= config_.ultrasonic_read_interval_ms) {
                if (config_.ultrasonic_callback) {
                    config_.ultrasonic_callback(ultrasonic_distances);
                }
                last_ultrasonic_wake = now;
            }

            if ((now - last_tof_wake) * portTICK_PERIOD_MS >= config_.tof_read_interval_ms) {
                if (config_.tof_callback) {
                    config_.tof_callback(tof_distance_mm, tof_valid, tof_status);
                }
                last_tof_wake = now;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "Continuous reading stopped");
}

void SensorControl::continuous_read_task(void* arg) {
    SensorControl* instance = static_cast<SensorControl*>(arg);
    if (instance) {
        instance->continuous_read_loop();
    }
    vTaskDelete(nullptr);
}

esp_err_t SensorControl::start_continuous() {
    if (continuous_mode_) {
        return ESP_OK;
    }

    if (!is_ready()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (tof_sensor_) {
        esp_err_t err = static_cast<TofSensorImpl*>(tof_sensor_)->start_continuous();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start TOF continuous mode");
            return err;
        }
    }

    continuous_mode_ = true;

    BaseType_t result = xTaskCreate(
        continuous_read_task,
        "sensor_read_task",
        4096,
        this,
        tskIDLE_PRIORITY + 1,
        &continuous_task_handle_
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create continuous reading task");
        continuous_mode_ = false;
        if (tof_sensor_) {
            static_cast<TofSensorImpl*>(tof_sensor_)->stop_continuous();
        }
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Continuous reading started");
    return ESP_OK;
}

esp_err_t SensorControl::stop_continuous() {
    if (!continuous_mode_) {
        return ESP_OK;
    }

    continuous_mode_ = false;
    vTaskDelay(pdMS_TO_TICKS(100));

    if (tof_sensor_) {
        static_cast<TofSensorImpl*>(tof_sensor_)->stop_continuous();
    }

    ESP_LOGI(TAG, "Continuous reading stopped");
    return ESP_OK;
}

bool SensorControl::is_continuous() const {
    return continuous_mode_;
}

esp_err_t SensorControl::set_tof_mode(bool long_distance) {
    if (!tof_sensor_) {
        return ESP_ERR_INVALID_STATE;
    }

    return static_cast<TofSensorImpl*>(tof_sensor_)->set_mode(long_distance);
}

size_t SensorControl::get_ultrasonic_count() const {
    return config_.ultrasonic_configs.size();
}

bool SensorControl::is_tof_ready() const {
    return tof_sensor_ && static_cast<TofSensorImpl*>(tof_sensor_)->is_ready();
}

bool SensorControl::is_ultrasonic_ready() const {
    return ultrasonic_array_ != nullptr;
}

esp_err_t SensorControl::self_test() {
    esp_err_t overall_result = ESP_OK;

    if (tof_sensor_) {
        bool probe_result = tof_probe();
        if (!probe_result) {
            ESP_LOGE(TAG, "TOF probe failed");
            overall_result = ESP_FAIL;
        } else {
            ESP_LOGI(TAG, "TOF probe passed");
        }
    }

    return overall_result;
}

bool SensorControl::tof_probe() {
    return tof_sensor_ && static_cast<TofSensorImpl*>(tof_sensor_)->probe();
}

// =============================================================================
// C-style interface implementation
// =============================================================================

extern "C" {

void sensor_control_init(void) {
    if (g_sensor_control != nullptr) {
        ESP_LOGW("sensor_control", "Already initialized");
        return;
    }

    // Create mutex for data access
    g_data_mutex = xSemaphoreCreateMutex();
    if (!g_data_mutex) {
        ESP_LOGE("sensor_control", "Failed to create data mutex");
        return;
    }

    // Configure with default ultrasonic sensors
    SensorControl::Config config;

    // Add 4 ultrasonic sensors (adjust pins as needed)
    config.ultrasonic_configs = {
        {.trig_pin = 25, .echo_pin = 26, .timeout_us = 30000, .max_distance_mm = 4000},
        {.trig_pin = 27, .echo_pin = 14, .timeout_us = 30000, .max_distance_mm = 4000},
        {.trig_pin = 12, .echo_pin = 13, .timeout_us = 30000, .max_distance_mm = 4000},
        {.trig_pin = 32, .echo_pin = 33, .timeout_us = 30000, .max_distance_mm = 4000}
    };

    config.ultrasonic_read_interval_ms = 100;
    config.tof_read_interval_ms = 200;

    g_sensor_control = new SensorControl(config);

    esp_err_t err = g_sensor_control->initialize();
    if (err != ESP_OK) {
        ESP_LOGE("sensor_control", "Initialization failed");
        delete g_sensor_control;
        g_sensor_control = nullptr;
        return;
    }

    ESP_LOGI("sensor_control", "Initialized successfully");
}

void sensor_control_start_task(void) {
    if (g_sensor_control == nullptr) {
        ESP_LOGE("sensor_control", "Not initialized - call sensor_control_init() first");
        return;
    }

    esp_err_t err = g_sensor_control->start_continuous();
    if (err != ESP_OK) {
        ESP_LOGE("sensor_control", "Failed to start continuous reading");
        return;
    }

    ESP_LOGI("sensor_control", "Continuous reading task started");
}

bool sensor_control_get_latest_data(SensorCommon::SensorDataPacket* data) {
    if (!data || !g_data_mutex) {
        return false;
    }

    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *data = g_latest_data;
        xSemaphoreGive(g_data_mutex);
        return true;
    }

    return false;
}

} // extern "C"
