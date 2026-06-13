#include <sensor_control.hpp>
#include <ultrasonic_sensor.hpp>
#include <tof_sensor.hpp>
#include <lidar_sensor.hpp>
#include <lidar_packet_parser.hpp>
#include <firmware_version.hpp>
#include <state_machine.hpp>
#include <sensor_manager.hpp>
#include <state_machine_lifecycle.hpp>

// Include adapter for the selected driver
#if SHELFBOT_DRIVER_VL53L0X
    #include "vl53l0x_adapter.hpp"
#elif SHELFBOT_DRIVER_VL53L1
    #include "vl53l1_adapter.hpp"
#elif SHELFBOT_DRIVER_VL53L1_MODBUS
    #include "vl53l1_modbus_adapter.hpp"
#elif SHELFBOT_DRIVER_LYDSTO
    #include "lydsto_adapter.hpp"
#endif

const char* SensorControl::TAG = "SensorControl";

// ----------------------------------------------------------------------------
// Lifecycle Task (same pattern as motor_control)
// ----------------------------------------------------------------------------
static void sensor_lifecycle_task(void* /*arg*/) {
    static const uint32_t RETRY_MS = 1000;
    ESP_LOGI(SensorControl::TAG, "Sensor lifecycle task started");

    // Step 1: Wait for the state to become IDLE (prerequisite: shelfbot running)
    while (!StateMachine::isInState("sensor_control", stateToString(SensorControlState::IDLE))) {
        StateMachine::advance("sensor_control");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }

    ESP_LOGI(SensorControl::TAG, "Sensor control now IDLE");

    // Step 2: Transition to SCANNING (prerequisite: wifi_manager connected)
    // The state machine rule for IDLE -> SCANNING requires wifi_manager == CONNECTED.
    // Wait until that is satisfied and advance.
    while (!StateMachine::isInState("sensor_control", stateToString(SensorControlState::SCANNING))) {
        StateMachine::advance("sensor_control");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }

    ESP_LOGI(SensorControl::TAG, "Sensor control now SCANNING – starting continuous reading");

    // Step 3: Start the continuous sensor reading loop (blocking)
    SensorManager::get_instance().start();

    // If start() ever returns (should not, unless error), keep the task alive
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ----------------------------------------------------------------------------
// SensorControl Implementation (unchanged except for the lifecycle task creation)
// ----------------------------------------------------------------------------

SensorControl::SensorControl(Config config) : config_(std::move(config)) {
    data_mutex_ = xSemaphoreCreateMutex();
    if (!data_mutex_) {
        ESP_LOGE(TAG, "Failed to create data mutex");
    }
}

SensorControl::~SensorControl() {
    stop_continuous();
    if (continuous_task_handle_) {
        vTaskDelete(continuous_task_handle_);
    }
    if (data_mutex_) {
        vSemaphoreDelete(data_mutex_);
    }
}

std::unique_ptr<IUltrasonicArray> SensorControl::createUltrasonicArray() {
#if SHELFBOT_HAS_ULTRASONIC == 0
    ESP_LOGW(TAG, "Ultrasonic support disabled at compile time");
    return nullptr;
#else
    if (config_.ultrasonic_configs.empty()) {
        ESP_LOGW(TAG, "No ultrasonic sensors configured");
        return nullptr;
    }
    auto array = std::make_unique<UltrasonicSensorArray>();
    for (size_t i = 0; i < config_.ultrasonic_configs.size(); ++i) {
        const auto& ucfg = config_.ultrasonic_configs[i];
        UltrasonicSensorConfig sensor_config = {
            .trig_pin = static_cast<gpio_num_t>(ucfg.trig_pin),
            .echo_pin = static_cast<gpio_num_t>(ucfg.echo_pin),
            .collision_threshold_cm = 20.0f,
            .timeout_us = ucfg.timeout_us,
            .state = SENSOR_IDLE,
            .start_time = 0,
            .pulse_duration = 0
        };
        if (!array->add_sensor(i, sensor_config)) {
            ESP_LOGE(TAG, "Failed to add ultrasonic sensor %zu", i);
            return nullptr;
        }
        ESP_LOGD(TAG, "Added ultrasonic sensor %zu: TRIG=GPIO%d, ECHO=GPIO%d",
                 i, ucfg.trig_pin, ucfg.echo_pin);
    }
    if (!array->init()) {
        ESP_LOGE(TAG, "Failed to initialize ultrasonic sensors");
        return nullptr;
    }
    ESP_LOGI(TAG, "Ultrasonic sensors initialized successfully");
    return array;
#endif
}

std::unique_ptr<IToFSensor> SensorControl::createToFSensor() {
#if SHELFBOT_HAS_TOF == 0
    ESP_LOGW(TAG, "ToF support disabled at compile time");
    return nullptr;
#else
    TofSensor::Config tof_config;
    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; ++i) {
        tof_config.tof_configs[i].device_id = config_.tof_configs[i].device_id;
        tof_config.tof_configs[i].timeout_ms = config_.tof_configs[i].timeout_ms;
        tof_config.tof_configs[i].enabled = config_.tof_configs[i].enabled;
    }
    tof_config.tof_read_interval_ms = config_.tof_read_interval_ms;
    tof_config.tof_callback = config_.tof_callback;

    auto sensor = std::make_unique<TofSensor>(tof_config);

    sensor->setDriverFactory([](uint8_t idx) -> std::unique_ptr<ITofDriver> {
        (void)idx;
#if SHELFBOT_DRIVER_VL53L0X
        return std::make_unique<VL53L0XAdapter>();
#elif SHELFBOT_DRIVER_VL53L1
        return std::make_unique<VL53L1Adapter>();
#elif SHELFBOT_DRIVER_VL53L1_MODBUS
        return std::make_unique<VL53L1ModbusAdapter>();
#elif SHELFBOT_DRIVER_LYDSTO
        return std::make_unique<LydstoAdapter>();
#else
#error "No ToF driver selected!"
#endif
    });

    if (sensor->initialize() != ESP_OK) {
        ESP_LOGE(TAG, "TOF sensor initialization failed");
        return nullptr;
    }
    ESP_LOGI(TAG, "TOF sensors initialized successfully");
    return sensor;
#endif
}

std::unique_ptr<ILidarSensor> SensorControl::createLidarSensor() {
#if SHELFBOT_HAS_LIDAR == 0
    ESP_LOGW(TAG, "LiDAR support disabled at compile time");
    return nullptr;
#else
    if (!config_.lidar_config.enabled) {
        ESP_LOGD(TAG, "LiDAR disabled in config");
        return nullptr;
    }
    auto lidar = std::make_unique<LidarSensor>(
        static_cast<uart_port_t>(config_.lidar_config.uart_port),
        config_.lidar_config.uart_tx_pin,
        config_.lidar_config.uart_rx_pin,
        config_.lidar_config.baud_rate);
    if (lidar->initialize() != ESP_OK) {
        ESP_LOGE(TAG, "LiDAR UART init failed; continuing without LiDAR");
        return nullptr;
    }
    ESP_LOGI(TAG, "LidarSensor initialized (UART=%d RX=%d TX=%d BAUD=%lu)",
             config_.lidar_config.uart_port,
             config_.lidar_config.uart_rx_pin,
             config_.lidar_config.uart_tx_pin,
             static_cast<unsigned long>(config_.lidar_config.baud_rate));
    return lidar;
#endif
}

esp_err_t SensorControl::initialize() {
    if (initialized_) return ESP_OK;

    ESP_LOGI(TAG, "Firmware Version: %s", FirmwareVersion::get_version_string());
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "Initializing Unified Sensor Control");
    ESP_LOGI(TAG, "=========================================");

    ultrasonic_array_ = createUltrasonicArray();
    tof_sensor_       = createToFSensor();
    lidar_sensor_     = createLidarSensor();

    for (int i = 0; i < SensorCommon::NUM_ULTRASONIC_SENSORS; ++i) {
        latest_data_.ultrasonic_readings[i].active =
            (ultrasonic_array_ && i < static_cast<int>(config_.ultrasonic_configs.size()));
    }
    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; ++i) {
        latest_data_.tof_measurements[i].active =
            (tof_sensor_ && config_.tof_configs[i].enabled);
    }
    latest_data_.lidar_measurement.active = (lidar_sensor_ != nullptr);

    initialized_ = true;

    // REMOVED: single StateMachine::advance("sensor_control") – lifecycle task handles it

    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "Sensor Control Initialization Complete");
    ESP_LOGI(TAG, "=========================================");
    return ESP_OK;
}

bool SensorControl::is_ready() const {
    bool ultrasonic_ready = config_.ultrasonic_configs.empty() || (ultrasonic_array_ != nullptr);
    bool tof_ready = (tof_sensor_ == nullptr) || tof_sensor_->isReady();
    return initialized_ && ultrasonic_ready && tof_ready;
}

esp_err_t SensorControl::read_ultrasonic(std::vector<uint16_t>& distances) {
    distances.clear();
    if (!ultrasonic_array_) return ESP_ERR_INVALID_STATE;

    std::vector<SensorCommon::Reading> readings;
    if (!ultrasonic_array_->readAll(readings, SensorCommon::DEFAULT_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "Failed to read ultrasonic sensors");
        return ESP_ERR_INVALID_RESPONSE;
    }
    for (const auto& reading : readings) {
        distances.push_back(static_cast<uint16_t>(reading.distance_cm * 10));
    }
    return ESP_OK;
}

esp_err_t SensorControl::read_tof(SensorCommon::TofMeasurement results[SensorCommon::NUM_TOF_SENSORS]) const {
    if (!tof_sensor_) return ESP_ERR_INVALID_STATE;
    return tof_sensor_->readAll(results);
}

esp_err_t SensorControl::read_lidar(SensorCommon::LidarMeasurement& result) const {
    if (!lidar_sensor_) return ESP_ERR_INVALID_STATE;
    return lidar_sensor_->read(result);
}

esp_err_t SensorControl::read_tof_single(uint8_t sensor_index, SensorCommon::TofMeasurement& result) const {
    if (!tof_sensor_) return ESP_ERR_INVALID_STATE;
    if (sensor_index >= SensorCommon::NUM_TOF_SENSORS) return ESP_ERR_INVALID_ARG;
    return tof_sensor_->readSingle(sensor_index, result);
}

esp_err_t SensorControl::read_all(std::vector<uint16_t>& ultrasonic_distances,
                                  SensorCommon::TofMeasurement tof_results[SensorCommon::NUM_TOF_SENSORS]) {
    esp_err_t err = ESP_OK;
    if (ultrasonic_array_) {
        if (read_ultrasonic(ultrasonic_distances) != ESP_OK) err = ESP_FAIL;
    }
    if (tof_sensor_) {
        if (read_tof(tof_results) != ESP_OK) err = ESP_FAIL;
    }
    return err;
}

void SensorControl::continuous_read_loop() {
    ESP_LOGI(TAG, "Starting continuous sensor reading...");
    TickType_t last_ultrasonic_wake = xTaskGetTickCount();
    TickType_t last_tof_wake        = xTaskGetTickCount();

    while (continuous_mode_) {
        TickType_t now = xTaskGetTickCount();

        SensorCommon::LidarMeasurement lidar_result;
        bool lidar_ok = false;
        if (lidar_sensor_) {
            lidar_ok = (lidar_sensor_->read(lidar_result) == ESP_OK);
        }

        if (lidar_ok && lidar_result.valid && config_.lidar_callback) {
            config_.lidar_callback(lidar_result);
        }

        if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
            const int64_t timestamp = esp_timer_get_time();

            std::vector<SensorCommon::Reading> ultrasonic_readings;
            if (ultrasonic_array_) {
                ultrasonic_array_->readAll(ultrasonic_readings, SensorCommon::DEFAULT_TIMEOUT_MS);
                for (size_t i = 0; i < ultrasonic_readings.size() && i < SensorCommon::NUM_ULTRASONIC_SENSORS; ++i) {
                    latest_data_.ultrasonic_readings[i] = ultrasonic_readings[i];
                }
            }

            if (tof_sensor_) {
                SensorCommon::TofMeasurement tof_results[SensorCommon::NUM_TOF_SENSORS];
                if (tof_sensor_->readAll(tof_results) == ESP_OK) {
                    for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; ++i) {
                        latest_data_.tof_measurements[i] = tof_results[i];
                    }
                }
            }

            if (lidar_ok) {
                latest_data_.lidar_measurement = lidar_result;
            }

            latest_data_.timestamp_us = timestamp;
            xSemaphoreGive(data_mutex_);

            if ((now - last_ultrasonic_wake) * portTICK_PERIOD_MS >= config_.ultrasonic_read_interval_ms) {
                if (config_.ultrasonic_callback && ultrasonic_array_) {
                    std::vector<uint16_t> distances_mm;
                    for (const auto& reading : ultrasonic_readings) {
                        distances_mm.push_back(static_cast<uint16_t>(reading.distance_cm * 10));
                    }
                    config_.ultrasonic_callback(distances_mm);
                }
                last_ultrasonic_wake = now;
            }

            if ((now - last_tof_wake) * portTICK_PERIOD_MS >= config_.tof_read_interval_ms) {
                if (config_.tof_callback && tof_sensor_) {
                    config_.tof_callback(latest_data_.tof_measurements);
                }
                last_tof_wake = now;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ESP_LOGI(TAG, "Continuous reading stopped");
}

void SensorControl::continuous_read_task(void* arg) {
    auto* instance = static_cast<SensorControl*>(arg);
    if (instance) instance->continuous_read_loop();
    vTaskDelete(nullptr);
}

esp_err_t SensorControl::start_continuous() {
    if (continuous_mode_) return ESP_OK;
    if (!is_ready()) return ESP_ERR_INVALID_STATE;

    if (tof_sensor_) {
        esp_err_t err = tof_sensor_->startContinuous();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start TOF continuous mode");
            return err;
        }
    }

    // Note: StateMachine::advance("sensor_control") is not called here;
    // the lifecycle task already advanced to SCANNING before calling start_continuous().
    continuous_mode_ = true;

    const BaseType_t result = xTaskCreate(
        continuous_read_task,
        "sensor_read_task",
        4096,
        this,
        tskIDLE_PRIORITY + 1,
        &continuous_task_handle_);

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create continuous reading task");
        continuous_mode_ = false;
        StateMachine::recover();
        if (tof_sensor_) tof_sensor_->stopContinuous();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Continuous reading started");
    return ESP_OK;
}

esp_err_t SensorControl::stop_continuous() {
    if (!continuous_mode_) return ESP_OK;
    continuous_mode_ = false;
    if (continuous_task_handle_) {
        vTaskDelay(pdMS_TO_TICKS(100));
        continuous_task_handle_ = nullptr;
    }
    if (tof_sensor_) tof_sensor_->stopContinuous();
    ESP_LOGI(TAG, "Continuous reading stopped");
    return ESP_OK;
}

bool SensorControl::is_continuous() const { return continuous_mode_; }

size_t SensorControl::get_ultrasonic_count() const {
    return config_.ultrasonic_configs.size();
}

bool SensorControl::is_tof_ready(uint8_t sensor_index) const {
    return tof_sensor_ && tof_sensor_->isSensorReady(sensor_index);
}

bool SensorControl::is_lidar_ready() const {
    return lidar_sensor_ && lidar_sensor_->isReady();
}

bool SensorControl::is_ultrasonic_ready() const {
    return ultrasonic_array_ != nullptr;
}

esp_err_t SensorControl::set_tof_mode(uint8_t sensor_index, bool long_distance) {
    (void)sensor_index;
    (void)long_distance;
    return ESP_OK;
}

esp_err_t SensorControl::self_test() const {
    esp_err_t overall_result = ESP_OK;
    if (tof_sensor_) {
        for (int i = 0; i < SensorCommon::NUM_TOF_SENSORS; ++i) {
            if (tof_sensor_->probe(i)) {
                ESP_LOGI(TAG, "TOF sensor %d probe passed", i);
            } else {
                ESP_LOGE(TAG, "TOF sensor %d probe failed", i);
                overall_result = ESP_FAIL;
            }
        }
    }
    return overall_result;
}

bool SensorControl::tof_probe(uint8_t sensor_index) const {
    return tof_sensor_ && tof_sensor_->probe(sensor_index);
}

uint8_t SensorControl::lidar_health() const {
    uint8_t health = 0;
    if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        health = latest_data_.lidar_measurement.health;
        xSemaphoreGive(data_mutex_);
    }
    return health;
}

bool SensorControl::get_last_lidar_raw_packet(uint8_t* out, size_t len) const {
    return lidar_sensor_ && lidar_sensor_->getLastRawPacket(out, len);
}

bool SensorControl::get_latest_data(SensorCommon::SensorDataPacket* data) const {
    if (!data || !data_mutex_) return false;
    if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        *data = latest_data_;
        xSemaphoreGive(data_mutex_);
        return true;
    }
    return false;
}

esp_err_t SensorControl::update_lidar_measurement() {
    return ESP_OK;
}

// ----------------------------------------------------------------------------
// Global lifecycle functions (called by the top-level shelfbot)
// ----------------------------------------------------------------------------

void sensor_control_setup() {
    ESP_LOGI(SensorControl::TAG, "sensor_control_setup called – creating lifecycle task");
    // Create the lifecycle task (same pattern as motor_control)
    xTaskCreate(sensor_lifecycle_task, "sensor_lifecycle", 4096, nullptr, 3, nullptr);
}

void sensor_control_init()   {}
void sensor_control_start()  {
    // The lifecycle task already started the continuous reading when state became SCANNING.
    // This function is kept for API consistency but does nothing.
    ESP_LOGD(SensorControl::TAG, "sensor_control_start called (no action needed)");
}
void sensor_control_stop()   { SensorManager::get_instance().stop(); }
void sensor_control_update() {}