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

// Forward declarations
static void orchestrator_task(void* arg);
static void network_services_task(void* arg);
static void time_sync_monitor_task(void* arg);
static void wifi_event_task(void* arg);

static std::unordered_map<std::string, std::string> get_current_states() {
    return {
        {"shelfbot", StateMachine::getState("shelfbot")},
        {"led_control", StateMachine::getState("led_control")},
        {"motor_control", StateMachine::getState("motor_control")},
        {"sensor_control", StateMachine::getState("sensor_control")},
        {"wifi_manager", StateMachine::getState("wifi_manager")},
        {"network_service", StateMachine::getState("network_service")},
        {"microros_sync", StateMachine::getState("microros_sync")},
        {"time_sync", StateMachine::getState("time_sync")},
        {"agent", StateMachine::getState("agent")},
    };
}

// Target states
static struct {
    std::string shelfbot = "running";
    std::string led = "running";
    std::string motor = "running";
    std::string sensor = "scanning";
    std::string wifi = "connected";
    std::string network = "http_running";
    std::string time = "synced";
    std::string microros = "connected";
    std::string agent = "connected";
} target;

static void try_transition(const std::string& module, const std::string& target_state,
                           std::function<void()> action) {
    auto current = get_current_states();
    std::string current_state = current[module];
    if (current_state == target_state) return;
    std::string next = get_next_allowed_state(module, target_state, current);
    if (next != current_state) {
        ESP_LOGI(TAG, "Transitioning %s: %s -> %s", module.c_str(), current_state.c_str(), next.c_str());
        action();
    }
}

static void orchestrator_task(void* arg) {
    while (true) {
        // 1. Allow WiFi and network tasks to update their states (they do internally)
        // 2. Move shelfbot from init to running once prerequisites are met
        try_transition("shelfbot", target.shelfbot, []() {
            StateMachine::changeState("shelfbot", stateToString(ShelfbotState::RUNNING));
        });

        // 3. LED and motor: advance sequentially
        std::string led_state = StateMachine::getState("led_control");
        if (led_state == "setup") {
            try_transition("led_control", "init", []() { led_control_setup(); });
        } else if (led_state == "init") {
            try_transition("led_control", "running", []() { led_control_init(); });
        } else if (led_state == "stopped") {
            try_transition("led_control", "setup", []() { led_control_start(); }); // restart
        }

        std::string motor_state = StateMachine::getState("motor_control");
        if (motor_state == "setup") {
            try_transition("motor_control", "init", []() { motor_control_setup(); });
        } else if (motor_state == "init") {
            try_transition("motor_control", "running", []() { motor_control_init(); });
        } else if (motor_state == "stopped") {
            try_transition("motor_control", "setup", []() { motor_control_start(); });
        }

        // 4. Sensor control
        std::string sensor_state = StateMachine::getState("sensor_control");
        if (sensor_state == "off") {
            try_transition("sensor_control", "idle", []() {
                sensor_control_setup();
                sensor_control_init();
                sensor_control_start(); // goes to scanning
            });
        } else if (sensor_state == "idle") {
            try_transition("sensor_control", "scanning", []() { sensor_control_start(); });
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// WiFi monitor task – waits for disconnection and forces rollback
static void wifi_event_task(void* arg) {
    EventGroupHandle_t wifi_evt = wifi_manager_get_event_group();
    if (!wifi_evt) {
        ESP_LOGE(TAG, "WiFi event group not available");
        vTaskDelete(nullptr);
        return;
    }
    while (true) {
        EventBits_t bits = xEventGroupWaitBits(wifi_evt, WM_CONNECTED_BIT | WM_DISCONNECTED_BIT,
                                               pdFALSE, pdFALSE, portMAX_DELAY);
        if (bits & WM_DISCONNECTED_BIT) {
            ESP_LOGW(TAG, "WiFi disconnected - forcing dependent modules to safe states");
            // Force network service back to OFF
            if (StateMachine::isInState("network_service", "http_running") ||
                StateMachine::isInState("network_service", "mdns_ready") ||
                StateMachine::isInState("network_service", "starting")) {
                StateMachine::changeState("network_service", stateToString(NetworkServiceState::OFF), true);
            }
            // Force time sync unsynced
            if (StateMachine::isInState("time_sync", "synced")) {
                StateMachine::changeState("time_sync", stateToString(TimeSyncState::UNSYNCED), true);
            }
            // Force microros and agent to DISCONNECTED/OFFLINE
            if (!StateMachine::isInState("microros_sync", stateToString(MicrorosState::DISCONNECTED))) {
                StateMachine::changeState("microros_sync", stateToString(MicrorosState::DISCONNECTED), true);
            }
            if (!StateMachine::isInState("agent", stateToString(AgentState::OFFLINE))) {
                StateMachine::changeState("agent", stateToString(AgentState::OFFLINE), true);
            }
            // Sensor control to IDLE
            if (StateMachine::isInState("sensor_control", "scanning")) {
                StateMachine::changeState("sensor_control", stateToString(SensorControlState::IDLE), true);
            }
        }
    }
}

// Network services task (unchanged)
static void network_services_task(void* arg) {
    EventGroupHandle_t wifi_evt = wifi_manager_get_event_group();
    if (!wifi_evt) {
        ESP_LOGE(TAG, "WiFi event group not available, aborting");
        while (true) vTaskDelay(pdMS_TO_TICKS(60000));
    }
    xEventGroupWaitBits(wifi_evt, WM_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    auto states = get_current_states();
    if (is_allowed_transition("network_service", stateToString(NetworkServiceState::STARTING), states)) {
        StateMachine::changeState("network_service", stateToString(NetworkServiceState::STARTING));
    } else {
        ESP_LOGE(TAG, "Cannot start network services – prerequisites not met");
        while (true) vTaskDelay(pdMS_TO_TICKS(10000));
    }

    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mdns_init failed: %s", esp_err_to_name(err));
        StateMachine::changeState("network_service", stateToString(NetworkServiceState::ERROR), true);
        while (true) vTaskDelay(pdMS_TO_TICKS(60000));
    }
    mdns_hostname_set("shelfbot");
    mdns_instance_name_set("Shelfbot ESP32 Client");
    mdns_service_add(nullptr, "_microros", "_udp", 8888, nullptr, 0);
    StateMachine::changeState("network_service", stateToString(NetworkServiceState::MDNS_READY));

    HttpServer::get_instance().start();
    StateMachine::changeState("network_service", stateToString(NetworkServiceState::HTTP_RUNNING));

    while (true) vTaskDelay(pdMS_TO_TICKS(60000));
}

// Time sync monitor task (unchanged but uses rank‑based check)
static void time_sync_monitor_task(void* arg) {
    while (true) {
        auto states = get_current_states();
        if (shelfbot::ShelfbotTimestamp::isEpochValid()) {
            if (is_allowed_transition("time_sync", stateToString(TimeSyncState::SYNCED), states)) {
                if (!StateMachine::isInState("time_sync", stateToString(TimeSyncState::SYNCED))) {
                    StateMachine::changeState("time_sync", stateToString(TimeSyncState::SYNCED), true);
                    ESP_LOGI(TAG, "Time sync achieved");
                }
            }
        } else {
            if (!StateMachine::isInState("time_sync", stateToString(TimeSyncState::UNSYNCED))) {
                StateMachine::changeState("time_sync", stateToString(TimeSyncState::UNSYNCED), true);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

Shelfbot& Shelfbot::get_instance() {
    if (!instance_) instance_ = new Shelfbot();
    return *instance_;
}

esp_err_t Shelfbot::begin() {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Shelfbot Firmware %s", FirmwareVersion::get_version_string());
    ESP_LOGI(TAG, "========================================");

    // Register modules
    StateMachine::setInitial("shelfbot", stateToString(ShelfbotState::SETUP), orderedStates(ShelfbotState()));
    StateMachine::setInitial("led_control", stateToString(LedControlState::SETUP), orderedStates(LedControlState()));
    StateMachine::setInitial("motor_control", stateToString(MotorControlState::SETUP), orderedStates(MotorControlState()));
    StateMachine::setInitial("sensor_control", stateToString(SensorControlState::OFF), orderedStates(SensorControlState()));
    StateMachine::setInitial("wifi_manager", stateToString(WifiManagerState::OFF), orderedStates(WifiManagerState()));
    StateMachine::setInitial("network_service", stateToString(NetworkServiceState::OFF), orderedStates(NetworkServiceState()));
    StateMachine::setInitial("microros_sync", stateToString(MicrorosState::DISCONNECTED), orderedStates(MicrorosState()));
    StateMachine::setInitial("time_sync", stateToString(TimeSyncState::UNSYNCED), orderedStates(TimeSyncState()));
    StateMachine::setInitial("agent", stateToString(AgentState::OFFLINE), orderedStates(AgentState()));

    StateMachine::init();

    // Start tasks
    xTaskCreate(orchestrator_task, "orchestrator", 4096, nullptr, 2, nullptr);
    xTaskCreate(time_sync_monitor_task, "time_sync_mon", 2048, nullptr, 3, nullptr);

    // NVS init
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) return ret;

    // WiFi
    wifi_manager_init();
    xTaskCreate(wifi_event_task, "wifi_monitor", 4096, nullptr, 4, nullptr);

    // Network services
    xTaskCreate(network_services_task, "net_services", 8192, nullptr, 5, nullptr);

    // micro-ROS
    MicrorosSync::getInstance().init();
    MicrorosSync::getInstance().start();

    // Sensors
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
    sensor_config.ultrasonic_callback = nullptr;
    sensor_config.tof_callback = nullptr;
    sensor_config.lidar_callback = nullptr;

    SensorManager::get_instance().initialize(sensor_config);
    SensorManager::get_instance().start();

    // Start shelfbot (move to INIT)
    StateMachine::changeState("shelfbot", stateToString(ShelfbotState::INIT));

    ESP_LOGI(TAG, "Initialization complete");
    return ESP_OK;
}