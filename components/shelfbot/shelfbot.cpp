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
#include <shelfbot_timestamp.hpp>

static const char* TAG = "Shelfbot";
Shelfbot* Shelfbot::instance_ = nullptr;

// ---------------------------------------------------------------------------
// network_services_task
// Owns: network_service   off -> starting -> mdns_ready -> http_running
// ---------------------------------------------------------------------------
static void network_services_task(void* /*arg*/) {
    static const uint32_t RETRY_MS = 1000;

    EventGroupHandle_t wifi_evt = wifi_manager_get_event_group();
    if (!wifi_evt) {
        ESP_LOGE(TAG, "WiFi event group unavailable – network services disabled");
        vTaskDelete(nullptr);
        return;
    }
    xEventGroupWaitBits(wifi_evt, WM_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    // off -> starting
    while (!StateMachine::canTransition("network_service",
                                        stateToString(NetworkServiceState::STARTING))) {
        ESP_LOGD(TAG, "net_services: waiting for prereqs: off->starting");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }
    StateMachine::changeState("network_service", stateToString(NetworkServiceState::STARTING));

    // starting -> mdns_ready
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mdns_init failed: %s", esp_err_to_name(err));
        StateMachine::changeState("network_service",
                                   stateToString(NetworkServiceState::ERROR), true);
        vTaskDelete(nullptr);
        return;
    }
    mdns_hostname_set("shelfbot");
    mdns_instance_name_set("Shelfbot ESP32 Client");
    mdns_service_add(nullptr, "_microros", "_udp", 8888, nullptr, 0);
    StateMachine::changeState("network_service", stateToString(NetworkServiceState::MDNS_READY));
    ESP_LOGI(TAG, "mDNS ready");

    // mdns_ready -> http_running
    HttpServer::get_instance().start();
    StateMachine::changeState("network_service", stateToString(NetworkServiceState::HTTP_RUNNING));
    ESP_LOGI(TAG, "HTTP server running");

    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// time_sync_monitor_task
// Owns: time_sync   unsynced <-> synced
// ---------------------------------------------------------------------------
static void time_sync_monitor_task(void* /*arg*/) {
    static const uint32_t POLL_MS = 2000;

    while (true) {
        const bool valid = shelfbot::ShelfbotTimestamp::isEpochValid();

        if (valid && !StateMachine::isInState("time_sync", "synced")) {
            if (StateMachine::canTransition("time_sync",
                                            stateToString(TimeSyncState::SYNCED))) {
                StateMachine::changeState("time_sync", stateToString(TimeSyncState::SYNCED));
                ESP_LOGI(TAG, "Time sync achieved");
            }
        } else if (!valid && StateMachine::isInState("time_sync", "synced")) {
            StateMachine::changeState("time_sync",
                                       stateToString(TimeSyncState::UNSYNCED), true);
            ESP_LOGW(TAG, "Time sync lost");
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

// ---------------------------------------------------------------------------
// wifi_event_task – forces dependent modules back to safe states on WiFi loss
// ---------------------------------------------------------------------------
static void wifi_event_task(void* /*arg*/) {
    EventGroupHandle_t wifi_evt = wifi_manager_get_event_group();
    if (!wifi_evt) { vTaskDelete(nullptr); return; }

    while (true) {
        xEventGroupWaitBits(wifi_evt,
                            WM_DISCONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

        ESP_LOGW(TAG, "WiFi lost – forcing dependent modules to safe states");

        const char* ns = StateMachine::getState("network_service").c_str();
        if (strcmp(ns, "off") != 0 && strcmp(ns, "error") != 0)
            StateMachine::changeState("network_service",
                                       stateToString(NetworkServiceState::OFF), true);

        if (StateMachine::isInState("time_sync", "synced"))
            StateMachine::changeState("time_sync",
                                       stateToString(TimeSyncState::UNSYNCED), true);

        if (!StateMachine::isInState("microros_sync", "disconnected") &&
            !StateMachine::isInState("microros_sync", "off"))
            StateMachine::changeState("microros_sync",
                                       stateToString(MicrorosState::DISCONNECTED), true);

        if (!StateMachine::isInState("agent", "offline"))
            StateMachine::changeState("agent",
                                       stateToString(AgentState::OFFLINE), true);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ---------------------------------------------------------------------------
// shelfbot_init_task – advances shelfbot: setup -> init -> running
// ---------------------------------------------------------------------------
static void shelfbot_init_task(void* /*arg*/) {
    static const uint32_t RETRY_MS = 1000;

    // setup -> init  (no prereqs)
    StateMachine::changeState("shelfbot", stateToString(ShelfbotState::INIT));
    ESP_LOGI(TAG, "Shelfbot init");

    // init -> running  (requires wifi:connected, network:mdns_ready, time:synced)
    while (!StateMachine::canTransition("shelfbot",
                                        stateToString(ShelfbotState::RUNNING))) {
        ESP_LOGD(TAG, "shelfbot: waiting for prereqs: init->running");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }
    StateMachine::changeState("shelfbot", stateToString(ShelfbotState::RUNNING));
    ESP_LOGI(TAG, "Shelfbot running");

    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// Shelfbot::begin
// ---------------------------------------------------------------------------
Shelfbot& Shelfbot::get_instance() {
    if (!instance_) instance_ = new Shelfbot();
    return *instance_;
}

esp_err_t Shelfbot::begin() {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Shelfbot Firmware %s", FirmwareVersion::get_version_string());
    ESP_LOGI(TAG, "========================================");

    // Register all modules with their initial state and ordered state list.
    StateMachine::setInitial("shelfbot",        stateToString(ShelfbotState::SETUP),        orderedStates(ShelfbotState()));
    StateMachine::setInitial("led_control",     stateToString(LedControlState::SETUP),      orderedStates(LedControlState()));
    StateMachine::setInitial("motor_control",   stateToString(MotorControlState::SETUP),    orderedStates(MotorControlState()));
    StateMachine::setInitial("sensor_control",  stateToString(SensorControlState::OFF),     orderedStates(SensorControlState()));
    StateMachine::setInitial("wifi_manager",    stateToString(WifiManagerState::OFF),       orderedStates(WifiManagerState()));
    StateMachine::setInitial("network_service", stateToString(NetworkServiceState::OFF),    orderedStates(NetworkServiceState()));
    StateMachine::setInitial("microros_sync",   stateToString(MicrorosState::DISCONNECTED), orderedStates(MicrorosState()));
    StateMachine::setInitial("time_sync",       stateToString(TimeSyncState::UNSYNCED),     orderedStates(TimeSyncState()));
    StateMachine::setInitial("agent",           stateToString(AgentState::OFFLINE),         orderedStates(AgentState()));

    StateMachine::init();

    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) return ret;

    // WiFi – has its own internal task that drives wifi_manager state.
    wifi_manager_init();

    // Monitoring tasks (lightweight, own their module's state progression)
    xTaskCreate(wifi_event_task,        "wifi_monitor",   2048, nullptr, 4, nullptr);
    xTaskCreate(network_services_task,  "net_services",   4096, nullptr, 5, nullptr);
    xTaskCreate(time_sync_monitor_task, "time_sync_mon",  2048, nullptr, 3, nullptr);
    xTaskCreate(shelfbot_init_task,     "shelfbot_init",  2048, nullptr, 2, nullptr);

    // micro-ROS – owns its own state progression internally.
    MicrorosSync::getInstance().init();
    MicrorosSync::getInstance().start();

    // Sensors – initialize hardware here; lifecycle task drives state.
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

    SensorManager::get_instance().initialize(sensor_config);

    // Kick off component lifecycle tasks (each drives itself forward).
    led_control_setup();      // spawns led_lifecycle task
    motor_control_setup();    // spawns motor_lifecycle task
    sensor_control_setup();   // spawns sensor_lifecycle task

    ESP_LOGI(TAG, "Initialization complete – components progressing independently");
    return ESP_OK;
}