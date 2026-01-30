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

// Read ToF Sensor via MODBUS
if (g_tof_sensor && g_tof_sensor->isReady()) {
  SensorCommon::Reading tof_reading;
  if (g_tof_sensor->readSingle(tof_reading)) {
    packet.tof_distances_cm[0] = tof_reading.distance_cm;
    packet.tof_valid[0] = tof_reading.valid;

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
      ESP_LOGW(TAG, "Obstacle: ToF[0] = %.1f cm (VALID, CLOSE)",
               tof_reading.distance_cm);
    }
  } else {
    ESP_LOGD(TAG, "ToF[0]: Failed to read");
  }
} else if (!g_tof_sensor) {
  if (log_counter == 0) {
    ESP_LOGD(TAG, "ToF sensor not initialized");
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
// Configure ToF Sensor via MODBUS
// ========================================
ESP_LOGI(TAG, "Configuring %d ToF sensor(s) via MODBUS...", NUM_TOF_SENSORS);
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
ESP_LOGE(TAG, "Failed to initialize TOF400F via MODBUS: %s", error);
g_tof_sensor = nullptr;  // Mark as unavailable
} else {
ESP_LOGI(TAG, "TOF400F MODBUS sensor initialized successfully");
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
// NOTE: ToF MODBUS sensor is read directly in sensor_reader_task
// No separate manager task needed
// Start unified sensor reader task
xTaskCreate(sensor_reader_task, "sensor_reader", 4096, NULL, 5, NULL);
ESP_LOGI(TAG, "Unified sensor reader task started");
}
bool sensor_control_get_latest_data(SensorDataPacket* packet) {
return xQueueReceive(unified_sensor_data_queue, packet, 0) == pdTRUE;
}
