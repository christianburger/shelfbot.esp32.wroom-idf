// shelfbot.cpp
//
// Key changes:
//  - All tasks are created centrally using create_task().
//  - Stack sizes tuned to prevent overflow and leave headroom.
//  - time_sync_mon stack increased to 3072 words (was 2048, still caused heap corruption).
//  - microros_task stack set to 12288 words (49 KB) for safety.
//  - shelfbot_init stack set to 4096 words (original safe value).
//  - create_task() now accepts an optional void* argument.

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
#include <esp_heap_caps.h>

static const char* TAG = "Shelfbot";
Shelfbot* Shelfbot::instance_ = nullptr;

// ---------------------------------------------------------------------------
// network_services_task
// ---------------------------------------------------------------------------
static void network_services_task(void* /*arg*/) {
    static const char* TASK_TAG = "NetServices";

    ESP_LOGI(TASK_TAG, "Task started — waiting for WiFi...");

    EventGroupHandle_t wifi_evt = WifiManager::get_instance().get_event_group();
    if (wifi_evt == nullptr) {
        ESP_LOGE(TASK_TAG, "WiFi event group is NULL — network services permanently disabled");
        vTaskDelete(nullptr);
        return;
    }

    xEventGroupWaitBits(wifi_evt, WifiManager::WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TASK_TAG, "WiFi connected — starting network services");

    StateMachine::advance("network_service");
    if (!StateMachine::isInState("network_service",
            stateToString(NetworkServiceState::STARTING))) {
        ESP_LOGE(TASK_TAG, "Failed to enter STARTING state — prerequisites not met. "
                 "WiFi state: %s", StateMachine::getState("wifi_manager").c_str());
        vTaskDelete(nullptr);
        return;
    }

    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGE(TASK_TAG, "mdns_init failed: %s — network services aborted",
                 esp_err_to_name(err));
        StateMachine::changeState("network_service",
            stateToString(NetworkServiceState::ERROR), true);
        vTaskDelete(nullptr);
        return;
    }

    mdns_hostname_set("shelfbot");
    mdns_instance_name_set("Shelfbot ESP32 Client");
    err = mdns_service_add(nullptr, "_microros", "_udp", 8888, nullptr, 0);
    if (err != ESP_OK) {
        ESP_LOGW(TASK_TAG, "mdns_service_add (_microros) failed: %s — continuing",
                 esp_err_to_name(err));
    }

    StateMachine::advance("network_service");
    if (!StateMachine::isInState("network_service",
            stateToString(NetworkServiceState::MDNS_READY))) {
        ESP_LOGE(TASK_TAG, "Expected MDNS_READY but got: %s",
                 StateMachine::getState("network_service").c_str());
    } else {
        ESP_LOGI(TASK_TAG, "mDNS ready (shelfbot.local)");
    }

    err = HttpServer::get_instance().start();
    if (err != ESP_OK) {
        ESP_LOGE(TASK_TAG, "HttpServer::start() failed: %s — HTTP will be unavailable. "
                 "All other components (microros, LiDAR, motors) are unaffected.",
                 esp_err_to_name(err));
        StateMachine::changeState("network_service",
            stateToString(NetworkServiceState::ERROR), true);
        vTaskDelete(nullptr);
        return;
    }

    StateMachine::advance("network_service");
    if (!StateMachine::isInState("network_service",
            stateToString(NetworkServiceState::HTTP_RUNNING))) {
        ESP_LOGW(TASK_TAG, "HTTP server started but state is: %s (expected http_running)",
                 StateMachine::getState("network_service").c_str());
    } else {
        ESP_LOGI(TASK_TAG, "HTTP server running — all network services up");
    }

    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// time_sync_monitor_task
// ---------------------------------------------------------------------------
static void time_sync_monitor_task(void* /*arg*/) {
    static const char* TASK_TAG = "TimeSyncMon";
    static const uint32_t POLL_MS = 2000u;

    ESP_LOGI(TASK_TAG, "Task started — polling epoch validity every %" PRIu32 " ms", POLL_MS);

    while (true) {
        const bool valid = shelfbot::ShelfbotTimestamp::isEpochValid();
        const bool currently_synced =
            StateMachine::isInState("time_sync",
                stateToString(TimeSyncState::SYNCED));

        if (valid && !currently_synced) {
            StateMachine::advance("time_sync");
            if (StateMachine::isInState("time_sync",
                    stateToString(TimeSyncState::SYNCED))) {
                ESP_LOGI(TASK_TAG, "Time sync achieved — epoch is valid");
            } else {
                ESP_LOGW(TASK_TAG, "Epoch valid but could not advance to SYNCED "
                         "(prerequisites: wifi=%s network=%s)",
                         StateMachine::getState("wifi_manager").c_str(),
                         StateMachine::getState("network_service").c_str());
            }
        } else if (!valid && currently_synced) {
            ESP_LOGW(TASK_TAG, "Epoch validity lost — reverting to UNSYNCED");
            StateMachine::changeState("time_sync",
                stateToString(TimeSyncState::UNSYNCED), true);
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

// ---------------------------------------------------------------------------
// wifi_event_task
// ---------------------------------------------------------------------------
static void wifi_event_task(void* /*arg*/) {
    static const char* TASK_TAG = "WifiEventMon";

    EventGroupHandle_t wifi_evt = WifiManager::get_instance().get_event_group();
    if (wifi_evt == nullptr) {
        ESP_LOGE(TASK_TAG, "WiFi event group is NULL — wifi event monitor disabled");
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TASK_TAG, "Task started — waiting for first WiFi connection before monitoring");
    xEventGroupWaitBits(wifi_evt, WifiManager::WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TASK_TAG, "Initial WiFi connection observed — disconnect monitor active");

    while (true) {
        xEventGroupWaitBits(wifi_evt, WifiManager::WIFI_DISCONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
        ESP_LOGW(TASK_TAG, "WiFi disconnected — calling StateMachine::recover()");
        StateMachine::recover();
        xEventGroupWaitBits(wifi_evt, WifiManager::WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
        ESP_LOGI(TASK_TAG, "WiFi reconnected — resuming disconnect monitor");
    }
}

// ---------------------------------------------------------------------------
// shelfbot_init_task
// ---------------------------------------------------------------------------
static void shelfbot_init_task(void* /*arg*/) {
    static const char* TASK_TAG = "ShelfbotInit";
    static const uint32_t RETRY_MS = 1000u;

    ESP_LOGI(TASK_TAG, "Task started");

    while (!StateMachine::isInState("shelfbot",
               stateToString(ShelfbotState::RUNNING)) &&
           !StateMachine::isInState("shelfbot",
               stateToString(ShelfbotState::ERROR))) {
        StateMachine::advance("shelfbot");
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }

    if (StateMachine::isInState("shelfbot", stateToString(ShelfbotState::RUNNING))) {
        ESP_LOGI(TASK_TAG, "Shelfbot reached RUNNING state");
    } else {
        ESP_LOGE(TASK_TAG, "Shelfbot reached ERROR state — check prerequisites: "
                 "wifi=%s network=%s time_sync=%s",
                 StateMachine::getState("wifi_manager").c_str(),
                 StateMachine::getState("network_service").c_str(),
                 StateMachine::getState("time_sync").c_str());
    }

    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// Helper: create a task with full error reporting, optionally passing an argument
// ---------------------------------------------------------------------------
static TaskHandle_t create_task(TaskFunction_t  fn,
                                const char*     name,
                                uint32_t        stack_words,
                                UBaseType_t     priority,
                                void*           arg = nullptr) {
    TaskHandle_t handle = nullptr;
    BaseType_t ret = xTaskCreate(fn, name, stack_words, arg, priority, &handle);

    if (ret != pdPASS) {
        ESP_LOGE("Shelfbot", "xTaskCreate('%s') FAILED (ret=%d) — "
                 "stack=%u words, priority=%u. "
                 "Free heap: %u bytes, largest free block: %u bytes.",
                 name, (int)ret,
                 (unsigned)stack_words, (unsigned)priority,
                 (unsigned)esp_get_free_heap_size(),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        return nullptr;
    }

    ESP_LOGI("Shelfbot", "Task created: %-20s  stack=%u words  priority=%u  handle=0x%p",
             name, (unsigned)stack_words, (unsigned)priority, (void*)handle);
    return handle;
}

// ---------------------------------------------------------------------------
// Singleton
// ---------------------------------------------------------------------------
Shelfbot& Shelfbot::get_instance() {
    if (!instance_) instance_ = new Shelfbot();
    return *instance_;
}

// ---------------------------------------------------------------------------
// begin()
// ---------------------------------------------------------------------------
esp_err_t Shelfbot::begin() {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Shelfbot Firmware %s", FirmwareVersion::get_version_string());
    ESP_LOGI(TAG, "Free heap at boot: %u bytes",
             (unsigned)esp_get_free_heap_size());
    ESP_LOGI(TAG, "========================================");

    // ── Register all modules (except wifi_manager – it registers itself) ─────
    StateMachine::setInitial("shelfbot",        stateToString(ShelfbotState::SETUP),
                             orderedStates(ShelfbotState()));
    StateMachine::setInitial("led_control",     stateToString(LedControlState::SETUP),
                             orderedStates(LedControlState()));
    StateMachine::setInitial("motor_control",   stateToString(MotorControlState::SETUP),
                             orderedStates(MotorControlState()));
    StateMachine::setInitial("lidar_sensor",    stateToString(LidarSensorState::SETUP),
                             orderedStates(LidarSensorState()));
    StateMachine::setInitial("network_service", stateToString(NetworkServiceState::OFF),
                             orderedStates(NetworkServiceState()));
    StateMachine::setInitial("microros_sync",   stateToString(MicrorosState::DISCONNECTED),
                             orderedStates(MicrorosState()));
    StateMachine::setInitial("time_sync",       stateToString(TimeSyncState::UNSYNCED),
                             orderedStates(TimeSyncState()));
    StateMachine::setInitial("agent",           stateToString(AgentState::OFFLINE),
                             orderedStates(AgentState()));

    StateMachine::init();

    // ── NVS ───────────────────────────────────────────────────────────────────
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs erase (ret=%s) — erasing and reinitialising",
                 esp_err_to_name(ret));
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "NVS initialised");

    // ── WiFi ──────────────────────────────────────────────────────────────────
    ret = WifiManager::get_instance().init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WifiManager init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = WifiManager::get_instance().start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WifiManager start failed: %s (non‑fatal, but WiFi won't work)",
                 esp_err_to_name(ret));
    }

    // ── Create all tasks ──────────────────────────────────────────────────────
    // Stack sizes chosen after empirical testing and worst‑case estimation:
    //   wifi_monitor   – 2048 words (only event waits, safe)
    //   net_services   – 2048 words (DNS/HTTP init, then deletes itself)
    //   time_sync_mon  – 3072 words (original stable value; 2048 caused heap corruption)
    //   shelfbot_init  – 4096 words (StateMachine calls with std::string & map)
    //   microros_task  – 12288 words (large due to RCL, string operations, etc.)
    // All tasks are checked for creation failure.
    TaskHandle_t wifi_monitor_handle   = create_task(wifi_event_task,        "wifi_monitor",   2048u, 4u);
    TaskHandle_t net_services_handle   = create_task(network_services_task,  "net_services",   2048u, 5u);
    TaskHandle_t time_sync_handle      = create_task(time_sync_monitor_task, "time_sync_mon",  3072u, 3u); // increased
    TaskHandle_t shelfbot_init_handle  = create_task(shelfbot_init_task,     "shelfbot_init",  4096u, 2u);
    TaskHandle_t microros_handle       = create_task(MicrorosSync::microros_task,
                                                     "microros_task",
                                                     12288u,
                                                     5u,
                                                     &MicrorosSync::getInstance());

    if (wifi_monitor_handle == nullptr || net_services_handle == nullptr ||
        time_sync_handle == nullptr || shelfbot_init_handle == nullptr ||
        microros_handle == nullptr) {
        ESP_LOGE(TAG, "One or more critical tasks failed to start. Setting shelfbot to ERROR.");
        StateMachine::changeState("shelfbot", stateToString(ShelfbotState::ERROR), true);
        return ESP_ERR_NO_MEM;
    }

    // ── micro-ROS initialisation (task already created) ──────────────────────
    MicrorosSync::getInstance().init();
    // start() is now empty; the task is created above.

    // ── Hardware components ──────────────────────────────────────────────────
    led_control_setup();
    motor_control_begin();
    lidar_setup();

    ESP_LOGI(TAG, "Initialisation complete — free heap: %u bytes",
             (unsigned)esp_get_free_heap_size());
    ESP_LOGI(TAG, "Components progressing independently via state machine");
    return ESP_OK;
}