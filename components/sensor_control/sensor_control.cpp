#include "sensor_control.hpp"
#include "ultrasonic_sensor.hpp"
#include "tof_sensor.hpp"
#include "esp_log.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include <vector>

static const char* TAG = "sensor_control";

// --- Configuration ---
const float COLLISION_THRESHOLD_CM = 2.0;
static int64_t last_emergency_stop_time_us = 0;
static const int64_t EMERGENCY_STOP_COOLDOWN_US = 500000; // 500ms cooldown

// --- Global Queues ---
QueueHandle_t unified_sensor_data_queue = nullptr;
QueueHandle_t motor_stop_queue = nullptr;
QueueHandle_t distance_data_queue = nullptr;  // Legacy queue

static void sensor_reader_task(void* arg) {
  std::vector<SensorCommon::Reading> ultrasonic_readings;
  std::vector<SensorCommon::Reading> tof_readings;
  SensorDataPacket packet;

  uint32_t log_counter = 0;
  const uint32_t LOG_INTERVAL = 10; // Log every 10th iteration (every ~1 second at 100ms interval)

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

        // Periodic logging of all readings
        if (log_counter == 0) {
          if (ultrasonic_readings[i].valid) {
            ESP_LOGI(TAG, "Ultrasonic[%zu]: %.1f cm", i, ultrasonic_readings[i].distance_cm);
          } else {
            ESP_LOGD(TAG, "Ultrasonic[%zu]: INVALID (%.1f cm)", i, ultrasonic_readings[i].distance_cm);
          }
        }

        // Only trigger on VALID, CLOSE readings
        if (ultrasonic_readings[i].valid &&
            ultrasonic_readings[i].distance_cm >= SensorCommon::MIN_DISTANCE_CM &&
            ultrasonic_readings[i].distance_cm < COLLISION_THRESHOLD_CM) {
          obstacle_detected = true;
          ESP_LOGW(TAG, "Obstacle: Ultrasonic[%zu] = %.1f cm (VALID, CLOSE)",
                   i, ultrasonic_readings[i].distance_cm);
        }
      }
    }

    // Read ToF Sensors
    if (ToFSensorManager::instance().get_latest_readings(tof_readings)) {
      for (size_t i = 0; i < tof_readings.size() && i < NUM_TOF_SENSORS; i++) {
        packet.tof_distances_cm[i] = tof_readings[i].distance_cm;
        packet.tof_valid[i] = tof_readings[i].valid;

        // Periodic logging of all readings
        if (log_counter == 0) {
          if (tof_readings[i].valid) {
            ESP_LOGI(TAG, "ToF[%zu]: %.1f cm", i, tof_readings[i].distance_cm);
          } else {
            ESP_LOGD(TAG, "ToF[%zu]: INVALID (%.1f cm)", i, tof_readings[i].distance_cm);
          }
        }

        // Only trigger on VALID, CLOSE readings
        if (tof_readings[i].valid &&
            tof_readings[i].distance_cm >= SensorCommon::MIN_DISTANCE_CM &&
            tof_readings[i].distance_cm < COLLISION_THRESHOLD_CM) {
          obstacle_detected = true;
          ESP_LOGW(TAG, "Obstacle: ToF[%zu] = %.1f cm (VALID, CLOSE)",
                   i, tof_readings[i].distance_cm);
        }
      }
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

// --- Public API Implementation ---
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
  // Configure ToF Sensors with Auto-Detection
  // ========================================
  ESP_LOGI(TAG, "Configuring %d ToF sensor(s) with auto-detection...", NUM_TOF_SENSORS);

  // NOTE: The ToF system will automatically:
  // 1. Send UART->I2C mode switch command (safe even without UART device)
  // 2. Scan I2C bus at specified address
  // 3. Detect VL53L0X or VL53L1 based on device ID
  // 4. Initialize the appropriate driver

  ToFSensorConfig tof_configs[NUM_TOF_SENSORS] = {
    {
      .i2c_port = I2C_NUM_0,
      .i2c_address = 0x29,           // Standard address for both VL53L0X and VL53L1
      .sda_pin = GPIO_NUM_21,
      .scl_pin = GPIO_NUM_22,
      .xshut_pin = GPIO_NUM_25,      // Optional shutdown pin

      // UART configuration for VL53L1 TOF400F mode switching
      // (Harmless if VL53L0X is connected instead)
      .uart_port = UART_NUM_1,
      .uart_tx_pin = GPIO_NUM_17,
      .uart_rx_pin = GPIO_NUM_16,

      // VL53L1 specific settings (used only if VL53L1 detected)
      .ranging_mode = VL53L1::RangingMode::HIGH_PRECISION,

      // VL53L0X specific settings (used only if VL53L0X detected)
      .io_2v8 = true,
      .timeout_ms = 500,
      .timing_budget_us = 200000,
      .signal_rate_limit_mcps = 0.25
    }
  };

  if (!ToFSensorManager::instance().configure(tof_configs, NUM_TOF_SENSORS)) {
    ESP_LOGE(TAG, "Failed to configure ToF sensors");
  } else {
    ESP_LOGI(TAG, "ToF sensors configured successfully");

    // Log detected sensor types
    auto detected_types = ToFSensorManager::instance().get_detected_types();
    for (size_t i = 0; i < detected_types.size(); i++) {
      const char* type_str = "NONE";
      switch (detected_types[i]) {
        case DetectedSensorType::VL53L0X:
          type_str = "VL53L0X";
          break;
        case DetectedSensorType::VL53L1:
          type_str = "VL53L1";
          break;
        default:
          break;
      }
      ESP_LOGI(TAG, "  Slot %zu: %s", i, type_str);
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

  // Start ToF reading task
  if (!ToFSensorManager::instance().start_reading_task(100, 5)) {
    ESP_LOGE(TAG, "Failed to start ToF reading task");
  } else {
    ESP_LOGI(TAG, "ToF reading task started");
  }

  // Start unified sensor reader task
  xTaskCreate(sensor_reader_task, "sensor_reader", 4096, NULL, 5, NULL);
  ESP_LOGI(TAG, "Unified sensor reader task started");
}

bool sensor_control_get_latest_data(SensorDataPacket* packet) {
  return xQueueReceive(unified_sensor_data_queue, packet, 0) == pdTRUE;
}
