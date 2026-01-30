#include "sensor_control.hpp"
#include "ultrasonic_sensor.hpp"
#include "tof_sensor.hpp"
#include "tof400f_modbus_sensor.hpp"
#include "esp_log.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include <vector>

static const char* TAG = "sensor_control";

// --- Configuration ---
const float COLLISION_THRESHOLD_CM = 2.0;
static int64_t last_emergency_stop_time_us = 0;
static const int64_t EMERGENCY_STOP_COOLDOWN_US = 500000;

// --- Global Queues ---
QueueHandle_t unified_sensor_data_queue = nullptr;
QueueHandle_t motor_stop_queue = nullptr;
QueueHandle_t distance_data_queue = nullptr;

// Global pointer to MODBUS ToF sensor
static Tof400fModbusSensor* g_tof_sensor = nullptr;

static void sensor_reader_task(void* arg) {
    std::vector<SensorCommon::Reading> ultrasonic_readings;

    SensorDataPacket packet;
    uint32_t log_counter = 0;
    const uint32_t LOG_INTERVAL = 10;

    for (;;) {
        bool obstacle_detected = false;
        int64_t current_time = esp_timer_get_time();
        packet.timestamp_us = current_time;

        // Initialize arrays to invalid values
        for (int i = 0; i < NUM_ULTRASONIC_SENSORS; i++) {
            packet.ultrasonic_distances_cm[i] = SensorCommon::MAX_DISTANCE_CM;
            packet.ultrasonic_valid[i] = false;
        }
        for (int i = 0; i < NUM_TOF_SENSORS; i++) {
            packet.tof_distances_cm[i] = SensorCommon::MAX_DISTANCE_CM;
            packet.tof_valid[i] = false;
        }

        // Read Ultrasonic Sensors
        if (UltrasonicSensorManager::instance().get_latest_readings(ultrasonic_readings)) {
            for (size_t i = 0; i < ultrasonic_readings.size() && i < NUM_ULTRASONIC_SENSORS; i++) {
                packet.ultrasonic_distances_cm[i] = ultrasonic_readings[i].distance_cm;
                packet.ultrasonic_valid[i] = ultrasonic_readings[i].valid;

                if (log_counter == 0) {
                    if (ultrasonic_readings[i].valid) {
                        ESP_LOGI(TAG, "Ultrasonic[%zu]: %.1f cm", i, ultrasonic_readings[i].distance_cm);
                    } else {
                        ESP_LOGD(TAG, "Ultrasonic[%zu]: INVALID (%.1f cm)", i, ultrasonic_readings[i].distance_cm);
                    }
                }

                if (ultrasonic_readings[i].valid &&
                    ultrasonic_readings[i].distance_cm >= SensorCommon::MIN_DISTANCE_CM &&
                    ultrasonic_readings[i].distance_cm < COLLISION_THRESHOLD_CM) {
                    obstacle_detected = true;
                    ESP_LOGW(TAG, "Obstacle: Ultrasonic[%zu] = %.1f cm (VALID, CLOSE)",
                             i, ultrasonic_readings[i].distance_cm);
                }
            }
        }

        // Read ToF Sensors (I2C has priority, MODBUS is fallback)
        bool tof_read_success = false;

        // Priority 1: Try I2C ToF sensors first
        std::vector<SensorCommon::Reading> tof_readings;
        if (ToFSensorManager::instance().get_latest_readings(tof_readings) && !tof_readings.empty()) {
            packet.tof_distances_cm[0] = tof_readings[0].distance_cm;
            packet.tof_valid[0] = tof_readings[0].valid;
            tof_read_success = true;

            if (log_counter == 0) {
                if (tof_readings[0].valid) {
                    ESP_LOGI(TAG, "ToF[0] (I2C): %.1f cm", tof_readings[0].distance_cm);
                } else {
                    ESP_LOGD(TAG, "ToF[0] (I2C): INVALID (%.1f cm)", tof_readings[0].distance_cm);
                }
            }

            if (tof_readings[0].valid &&
                tof_readings[0].distance_cm >= SensorCommon::MIN_DISTANCE_CM &&
                tof_readings[0].distance_cm < COLLISION_THRESHOLD_CM) {
                obstacle_detected = true;
                ESP_LOGW(TAG, "Obstacle: ToF[0] = %.1f cm (VALID, CLOSE)", tof_readings[0].distance_cm);
            }
        }

        // Priority 2: If I2C not available, try MODBUS sensor
        if (!tof_read_success && g_tof_sensor && g_tof_sensor->isReady()) {
            SensorCommon::Reading tof_reading;
            if (g_tof_sensor->readSingle(tof_reading)) {
                packet.tof_distances_cm[0] = tof_reading.distance_cm;
                packet.tof_valid[0] = tof_reading.valid;
                tof_read_success = true;

                if (log_counter == 0) {
                    if (tof_reading.valid) {
                        ESP_LOGI(TAG, "ToF[0] (MODBUS): %.1f cm", tof_reading.distance_cm);
                    } else {
                        ESP_LOGD(TAG, "ToF[0] (MODBUS): INVALID (%.1f cm)", tof_reading.distance_cm);
                    }
                }

                if (tof_reading.valid &&
                    tof_reading.distance_cm >= SensorCommon::MIN_DISTANCE_CM &&
                    tof_reading.distance_cm < COLLISION_THRESHOLD_CM) {
                    obstacle_detected = true;
                    ESP_LOGW(TAG, "Obstacle: ToF[0] = %.1f cm (VALID, CLOSE)", tof_reading.distance_cm);
                }
            }
        }

        if (!tof_read_success && log_counter == 0) {
            ESP_LOGD(TAG, "No ToF sensors available (neither MODBUS nor I2C)");
        }

        // Increment and wrap log counter
        log_counter = (log_counter + 1) % LOG_INTERVAL;

        // Send unified data to queue
        xQueueOverwrite(unified_sensor_data_queue, &packet);

        // Legacy queue support
        float legacy_distances[NUM_ULTRASONIC_SENSORS];
        for (int i = 0; i < NUM_ULTRASONIC_SENSORS; i++) {
            legacy_distances[i] = packet.ultrasonic_distances_cm[i];
        }
        xQueueOverwrite(distance_data_queue, &legacy_distances);

        // Emergency stop with cooldown
        if (obstacle_detected) {
            int64_t time_since_last_stop = current_time - last_emergency_stop_time_us;

            if (time_since_last_stop >= EMERGENCY_STOP_COOLDOWN_US) {
                int stop_signal = 1;
                xQueueSend(motor_stop_queue, &stop_signal, 0);
                last_emergency_stop_time_us = current_time;
                ESP_LOGW(TAG, "Emergency stop triggered - valid obstacle within %.1f cm",
                         COLLISION_THRESHOLD_CM);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void sensor_control_init() {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Initializing Unified Sensor Control");
    ESP_LOGI(TAG, "========================================");

    // Create queues
    unified_sensor_data_queue = xQueueCreate(1, sizeof(SensorDataPacket));
    motor_stop_queue = xQueueCreate(1, sizeof(int));
    distance_data_queue = xQueueCreate(1, sizeof(float[NUM_ULTRASONIC_SENSORS]));

    // ========================================
    // Configure Ultrasonic Sensors
    // ========================================
    ESP_LOGI(TAG, "Configuring %d ultrasonic sensor(s)...", NUM_ULTRASONIC_SENSORS);

    UltrasonicSensorConfig ultrasonic_configs[NUM_ULTRASONIC_SENSORS] = {
        {
            .trig_pin = GPIO_NUM_32,
            .echo_pin = GPIO_NUM_34,
            .collision_threshold_cm = COLLISION_THRESHOLD_CM,
            .timeout_us = 30000
        },
        {
            .trig_pin = GPIO_NUM_5,
            .echo_pin = GPIO_NUM_35,
            .collision_threshold_cm = COLLISION_THRESHOLD_CM,
            .timeout_us = 30000
        }
    };

    if (!UltrasonicSensorManager::instance().configure(ultrasonic_configs, NUM_ULTRASONIC_SENSORS)) {
        ESP_LOGE(TAG, "Failed to configure ultrasonic sensors");
    } else {
        ESP_LOGI(TAG, "Ultrasonic sensors configured successfully");
    }

    // ========================================
    // Configure ToF Sensors (I2C first, then MODBUS fallback)
    // ========================================
    ESP_LOGI(TAG, "Configuring %d ToF sensor(s)...", NUM_TOF_SENSORS);

    // Priority 1: Scan I2C bus for VL53L0X/VL53L1 sensors
    ESP_LOGI(TAG, "Scanning I2C bus for ToF sensors...");

    ToFSensorConfig tof_configs[NUM_TOF_SENSORS] = {
        {
            .i2c_port = I2C_NUM_0,
            .i2c_address = 0x29,
            .sda_pin = GPIO_NUM_21,
            .scl_pin = GPIO_NUM_22,
            .xshut_pin = GPIO_NUM_25,
            .uart_port = UART_NUM_1,
            .uart_tx_pin = GPIO_NUM_17,
            .uart_rx_pin = GPIO_NUM_16,
            .ranging_mode = VL53L1::RangingMode::HIGH_PRECISION,
            .io_2v8 = true,
            .timeout_ms = 500,
            .timing_budget_us = 200000,
            .signal_rate_limit_mcps = 0.25
        }
    };

    bool i2c_sensor_found = ToFSensorManager::instance().configure(tof_configs, NUM_TOF_SENSORS);

    if (i2c_sensor_found) {
        ESP_LOGI(TAG, "I2C ToF sensor found and configured - using I2C");
        g_tof_sensor = nullptr;  // Don't use MODBUS
    } else {
        // Priority 2: No I2C sensor found, try MODBUS TOF400F as fallback
        ESP_LOGI(TAG, "No I2C ToF sensors found, trying MODBUS...");

        static Tof400fModbusSensor::Config tof_modbus_config = {
            .uart_port = UART_NUM_1,
            .tx_pin = GPIO_NUM_17,
            .rx_pin = GPIO_NUM_16,
            .baud_rate = 115200,
            .modbus_address = 0x01,
            .ranging_mode = Tof400fModbus::RangingMode::HIGH_PRECISION,
            .timeout_ms = 500
        };

        static Tof400fModbusSensor tof_sensor_instance(tof_modbus_config);
        g_tof_sensor = &tof_sensor_instance;

        const char* error = g_tof_sensor->init();
        if (error) {
            ESP_LOGW(TAG, "MODBUS TOF400F not available: %s", error);
            g_tof_sensor = nullptr;
            ESP_LOGW(TAG, "No ToF sensors found (neither I2C nor MODBUS)");
        } else {
            ESP_LOGI(TAG, "MODBUS TOF400F sensor configured - using MODBUS");
        }
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Sensor Control Initialization Complete");
    ESP_LOGI(TAG, "========================================");
}

void sensor_control_start_task() {
    ESP_LOGI(TAG, "Starting sensor reading tasks...");

    // Start ultrasonic reading task
    if (!UltrasonicSensorManager::instance().start_reading_task(100, 5)) {
        ESP_LOGE(TAG, "Failed to start ultrasonic reading task");
    } else {
        ESP_LOGI(TAG, "Ultrasonic reading task started");
    }

    // Start I2C ToF reading task ONLY if I2C sensors configured (not MODBUS)
    if (!g_tof_sensor) {  // If g_tof_sensor is null, we're using I2C
        if (!ToFSensorManager::instance().start_reading_task(400, 5)) {
            ESP_LOGD(TAG, "I2C ToF reading task not started (no sensors)");
        } else {
            ESP_LOGI(TAG, "I2C ToF reading task started (400ms interval)");
        }
    }

    // Start unified sensor reader task
    xTaskCreate(sensor_reader_task, "sensor_reader", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Unified sensor reader task started");
}

bool sensor_control_get_latest_data(SensorDataPacket* packet) {
    return xQueueReceive(unified_sensor_data_queue, packet, 0) == pdTRUE;
}
