#include <shelfbot.hpp>
#include <microros_sync.hpp>
#include <sensor_manager.hpp>
#include <motor_control.hpp>
#include <led_control.hpp>
#include <wifi_manager.hpp>
#include <http_server.hpp>
#include <firmware_version.hpp>
#include <state_machine.hpp>
#include <state_machine_lifecycle.hpp>

static const char* TAG = "Shelfbot";

Shelfbot* Shelfbot::instance_ = nullptr;

Shelfbot& Shelfbot::get_instance() {
    if (!instance_) instance_ = new Shelfbot();
    return *instance_;
}

static void network_services_task(void* arg) {
    EventGroupHandle_t wifi_evt = wifi_manager_get_event_group();
    xEventGroupWaitBits(wifi_evt, WM_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    mdns_init();
    mdns_hostname_set("shelfbot");
    mdns_instance_name_set("Shelfbot ESP32 Client");

    HttpServer::get_instance().start();

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    while (1) vTaskDelay(pdMS_TO_TICKS(60000));
}

esp_err_t Shelfbot::begin() {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Shelfbot Firmware v%s", FirmwareVersion::get_version_string());
    ESP_LOGI(TAG, "========================================");

    // Initialize central state machine first
    StateMachine::init();
    StateMachine::setInitial("shelfbot", stateToString(ShelfbotState::STARTING));

    // 1. Hardware
    led_control_init();
    motor_control_begin();   // internally calls StateMachine::setInitial for motor_control

    // 2. NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) return ret;

    // 3. Wi‑Fi
    ret = wifi_manager_init(); // internally calls StateMachine::setInitial for wifi_manager
    if (ret != ESP_OK) return ret;

    // 4. Sensors
    SensorControl::Config sensor_config;
    sensor_config.ultrasonic_configs = {
        {.trig_pin = 25, .echo_pin = 34, .timeout_us = 30000, .max_distance_mm = 4000},
        {.trig_pin = 32, .echo_pin = 35, .timeout_us = 30000, .max_distance_mm = 4000},
        {.trig_pin = 16, .echo_pin = 36, .timeout_us = 30000, .max_distance_mm = 4000},
        {.trig_pin = 17, .echo_pin = 39, .timeout_us = 30000, .max_distance_mm = 4000},
    };
    sensor_config.lidar_config.enabled = true;
    sensor_config.lidar_config.uart_port = UART_NUM_2;
    sensor_config.lidar_config.uart_tx_pin = UART_PIN_NO_CHANGE;
    sensor_config.lidar_config.uart_rx_pin = GPIO_NUM_3;
    sensor_config.lidar_config.baud_rate = 115200;

    sensor_config.ultrasonic_callback = [](const std::vector<uint16_t>& distances) {
        float float_distances[SensorCommon::NUM_ULTRASONIC_SENSORS];
        for (size_t i = 0; i < distances.size(); ++i)
            float_distances[i] = distances[i] / 10.0f;
        MicrorosSync::publishDistanceSensors(float_distances, distances.size());
    };
    sensor_config.tof_callback = [](const SensorCommon::TofMeasurement* measurements) {
        float dist_m = measurements[0].valid ? measurements[0].distance_mm / 1000.0f : -1.0f;
        MicrorosSync::publishTofDistance(dist_m);
    };
    sensor_config.lidar_callback = [](const SensorCommon::LidarMeasurement& measurement) {
        MicrorosSync::publishLidarScan(measurement);
    };

    SensorManager::get_instance().initialize(sensor_config);
    SensorManager::get_instance().start(); // internally calls StateMachine::setInitial for sensor_control

    // 5. Network services task
    xTaskCreate(network_services_task, "net_services", 8192, nullptr, 5, nullptr);

    // 6. Micro-ROS node
    MicrorosSync::getInstance().init();  // calls StateMachine::setInitial for microros_sync
    MicrorosSync::getInstance().start();

    // Shelfbot now running
    StateMachine::changeState("shelfbot", stateToString(ShelfbotState::RUNNING));

    ESP_LOGI(TAG, "Initialization complete");
    return ESP_OK;
}