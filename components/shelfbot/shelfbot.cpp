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

// ---------------------------------------------------------------------------
// Network services task
// Starts after Wi-Fi is connected (waits on WM_CONNECTED_BIT).
// Responsibilities:
//  - Register itself in the state machine.
//  - Initialise mDNS (once, idempotent)
//  - Start HTTP server
//
// SNTP is now started by wifi_manager's on_ip_event handler the moment we
// obtain an IP address, giving it the maximum possible time to settle before
// microros_sync reaches its TIME_SYNCING state.  We do NOT start it here.
// ---------------------------------------------------------------------------
static void network_services_task(void* arg) {
    EventGroupHandle_t wifi_evt = wifi_manager_get_event_group();
    xEventGroupWaitBits(wifi_evt, WM_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    // Register this module and announce we are starting
    StateMachine::setInitial("network_service", stateToString(NetworkServiceState::STARTING));

    // mDNS initialisation
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mdns_init failed: %s", esp_err_to_name(err));
        StateMachine::changeState("network_service", stateToString(NetworkServiceState::ERROR));
        while (true) vTaskDelay(pdMS_TO_TICKS(60000));
    }
    mdns_hostname_set("shelfbot");
    mdns_instance_name_set("Shelfbot ESP32 Client");
    mdns_service_add(nullptr, "_microros", "_udp", 8888, nullptr, 0);
    StateMachine::changeState("network_service", stateToString(NetworkServiceState::MDNS_READY));

    // HTTP server
    HttpServer::get_instance().start();
    StateMachine::changeState("network_service", stateToString(NetworkServiceState::HTTP_RUNNING));

    while (true) vTaskDelay(pdMS_TO_TICKS(60000));
}

// ---------------------------------------------------------------------------
// Shelfbot::begin
// ---------------------------------------------------------------------------
esp_err_t Shelfbot::begin() {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Shelfbot Firmware %s", FirmwareVersion::get_version_string());
    ESP_LOGI(TAG, "Build %lu", (unsigned long)FirmwareVersion::get_build());
    ESP_LOGI(TAG, "========================================");

    // --- State machine framework (must be first) ---
    StateMachine::init();
    StateMachine::setInitial("shelfbot", stateToString(ShelfbotState::STARTING));

    // --- 1. Hardware peripherals ---
    led_control_init();
    motor_control_begin();   // registers "motor_control" in the state machine

    // --- 2. NVS ---
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // --- 3. Wi-Fi ---
    // wifi_manager_init() also:
    //   • registers "wifi_manager" in the state machine
    //   • starts SNTP via on_ip_event the moment we get an IP
    ret = wifi_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // --- 4. Sensors ---
    SensorControl::Config sensor_config;
    sensor_config.ultrasonic_configs = {
        {.trig_pin = 25, .echo_pin = 34, .timeout_us = 30000, .max_distance_mm = 4000},
        {.trig_pin = 32, .echo_pin = 35, .timeout_us = 30000, .max_distance_mm = 4000},
        {.trig_pin = 16, .echo_pin = 36, .timeout_us = 30000, .max_distance_mm = 4000},
        {.trig_pin = 17, .echo_pin = 39, .timeout_us = 30000, .max_distance_mm = 4000},
    };
    sensor_config.lidar_config.enabled     = true;
    sensor_config.lidar_config.uart_port   = UART_NUM_2;
    sensor_config.lidar_config.uart_tx_pin = UART_PIN_NO_CHANGE;
    sensor_config.lidar_config.uart_rx_pin = GPIO_NUM_3;
    sensor_config.lidar_config.baud_rate   = 115200;

    // Sensor callbacks — these feed into the microros_sync publish helpers
    // which gate on time_synced internally, so no guard is needed here.
    sensor_config.ultrasonic_callback = [](const std::vector<uint16_t>& distances) {
        float float_distances[SensorCommon::NUM_ULTRASONIC_SENSORS];
        for (size_t i = 0; i < distances.size(); ++i)
            float_distances[i] = distances[i] / 10.0f;
        MicrorosSync::publishDistanceSensors(float_distances, distances.size());
    };
    sensor_config.tof_callback = [](const SensorCommon::TofMeasurement* measurements) {
        float dist_m = measurements[0].valid
            ? measurements[0].distance_mm / 1000.0f : -1.0f;
        MicrorosSync::publishTofDistance(dist_m);
    };
    sensor_config.lidar_callback = [](const SensorCommon::LidarMeasurement& measurement) {
        // publishLidarScan is a no-op stub; the sensor_control_timer in
        // MicrorosSyncImpl publishes LaserScan with proper timestamps.
        MicrorosSync::publishLidarScan(measurement);
    };

    SensorManager::get_instance().initialize(sensor_config); // registers "sensor_control"
    SensorManager::get_instance().start();

    // --- 5. Network services (mDNS + HTTP) — runs after Wi-Fi connects ---
    xTaskCreate(network_services_task, "net_services", 8192, nullptr, 5, nullptr);

    // --- 6. micro-ROS ---
    // MicrorosSync::init() registers "microros_sync" in the state machine.
    MicrorosSync::getInstance().init();
    MicrorosSync::getInstance().start();

    // --- Done ---
    StateMachine::changeState("shelfbot", stateToString(ShelfbotState::RUNNING));
    ESP_LOGI(TAG, "Initialization complete");
    return ESP_OK;
}