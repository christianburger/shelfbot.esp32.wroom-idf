#include <shelfbot.hpp>
#include <microros_sync.hpp>
#include <motor_control.hpp>
#include <led_control.hpp>
#include <wifi_manager.hpp>
#include <http_server.hpp>
#include <firmware_version.hpp>
#include <state_machine.hpp>
#include <state_machine_lifecycle.hpp>
#include <shelfbot_timestamp.hpp>
#include <lidar_sensor.hpp>

static const char* TAG = "Shelfbot";
Shelfbot* Shelfbot::instance_ = nullptr;

static void network_services_task(void* /*arg*/) {
    static const uint32_t RETRY_MS = 1000;
    EventGroupHandle_t wifi_evt = wifi_manager_get_event_group();
    if (!wifi_evt) {
        ESP_LOGE(TAG, "WiFi event group unavailable – network services disabled");
        vTaskDelete(nullptr);
        return;
    }
    xEventGroupWaitBits(wifi_evt, WM_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    while (!StateMachine::isInState("network_service", stateToString(NetworkServiceState::STARTING))) {
        StateMachine::advance("network_service");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }

    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mdns_init failed: %s", esp_err_to_name(err));
        StateMachine::recover();
        vTaskDelete(nullptr);
        return;
    }
    mdns_hostname_set("shelfbot");
    mdns_instance_name_set("Shelfbot ESP32 Client");
    mdns_service_add(nullptr, "_microros", "_udp", 8888, nullptr, 0);

    StateMachine::advance("network_service");
    ESP_LOGI(TAG, "mDNS ready");

    while (!StateMachine::isInState("network_service", stateToString(NetworkServiceState::MDNS_READY))) {
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }

    HttpServer::get_instance().start();
    StateMachine::advance("network_service");
    ESP_LOGI(TAG, "HTTP server running");
    vTaskDelete(nullptr);
}

static void time_sync_monitor_task(void* /*arg*/) {
    static const uint32_t POLL_MS = 2000;
    while (true) {
        const bool valid = shelfbot::ShelfbotTimestamp::isEpochValid();
        if (valid && !StateMachine::isInState("time_sync", "synced")) {
            StateMachine::advance("time_sync");
            if (StateMachine::isInState("time_sync", "synced")) {
                ESP_LOGI(TAG, "Time sync achieved");
            }
        } else if (!valid && StateMachine::isInState("time_sync", "synced")) {
            StateMachine::recover();
            ESP_LOGW(TAG, "Time sync lost");
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

static void wifi_event_task(void* /*arg*/) {
    EventGroupHandle_t wifi_evt = wifi_manager_get_event_group();
    if (!wifi_evt) { vTaskDelete(nullptr); return; }
    while (true) {
        xEventGroupWaitBits(wifi_evt, WM_DISCONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
        ESP_LOGW(TAG, "WiFi lost – forcing dependent modules to safe states");
        StateMachine::recover();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void shelfbot_init_task(void* /*arg*/) {
    static const uint32_t RETRY_MS = 1000;
    StateMachine::advance("shelfbot");
    ESP_LOGI(TAG, "Shelfbot init");
    while (!StateMachine::isInState("shelfbot", stateToString(ShelfbotState::RUNNING))) {
        StateMachine::advance("shelfbot");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }
    ESP_LOGI(TAG, "Shelfbot running");
    vTaskDelete(nullptr);
}

Shelfbot& Shelfbot::get_instance() {
    if (!instance_) instance_ = new Shelfbot();
    return *instance_;
}

esp_err_t Shelfbot::begin() {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Shelfbot Firmware %s", FirmwareVersion::get_version_string());
    ESP_LOGI(TAG, "========================================");

    StateMachine::setInitial("shelfbot",        stateToString(ShelfbotState::SETUP),        orderedStates(ShelfbotState()));
    StateMachine::setInitial("led_control",     stateToString(LedControlState::SETUP),      orderedStates(LedControlState()));
    StateMachine::setInitial("motor_control",   stateToString(MotorControlState::SETUP),    orderedStates(MotorControlState()));
    StateMachine::setInitial("lidar_sensor",    stateToString(LidarSensorState::SETUP),     orderedStates(LidarSensorState()));
    StateMachine::setInitial("wifi_manager",    stateToString(WifiManagerState::OFF),       orderedStates(WifiManagerState()));
    StateMachine::setInitial("network_service", stateToString(NetworkServiceState::OFF),    orderedStates(NetworkServiceState()));
    StateMachine::setInitial("microros_sync",   stateToString(MicrorosState::DISCONNECTED), orderedStates(MicrorosState()));
    StateMachine::setInitial("time_sync",       stateToString(TimeSyncState::UNSYNCED),     orderedStates(TimeSyncState()));
    StateMachine::setInitial("agent",           stateToString(AgentState::OFFLINE),         orderedStates(AgentState()));

    StateMachine::init();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) return ret;

    wifi_manager_init();

    xTaskCreate(wifi_event_task,        "wifi_monitor",   2048, nullptr, 4, nullptr);
    xTaskCreate(network_services_task,  "net_services",   4096, nullptr, 5, nullptr);
    xTaskCreate(time_sync_monitor_task, "time_sync_mon",  2048, nullptr, 3, nullptr);
    xTaskCreate(shelfbot_init_task,     "shelfbot_init",  2048, nullptr, 2, nullptr);

    MicrorosSync::getInstance().init();
    MicrorosSync::getInstance().start();

    led_control_setup();
    motor_control_begin();
    lidar_setup();

    ESP_LOGI(TAG, "Initialization complete – components progressing independently");
    return ESP_OK;
}