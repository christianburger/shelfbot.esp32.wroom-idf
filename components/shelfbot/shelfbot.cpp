// shelfbot.cpp
//
// ALL FreeRTOS task creation happens here — no other component spawns tasks.
// This gives one auditable location for the entire heap/stack budget.

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

// ---------------------------------------------------------------------------
// network_services_task
// ---------------------------------------------------------------------------
static void network_services_task(void* /*arg*/) {
    static const char* TASK_TAG = "NetServices";

    ESP_LOGI(TASK_TAG, "Task started — waiting for WiFi...");

    EventGroupHandle_t wifi_evt = wifi_manager_get_event_group();
    if (wifi_evt == nullptr) {
        ESP_LOGE(TASK_TAG, "WiFi event group is NULL — network services permanently disabled");
        vTaskDelete(nullptr);
        return;
    }

    xEventGroupWaitBits(wifi_evt, WM_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
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
        ESP_LOGE(TASK_TAG, "HttpServer::start() failed: %s — HTTP will be unavailable.",
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
[[noreturn]] static void time_sync_monitor_task(void* /*arg*/) {
    static const char* TASK_TAG = "TimeSyncMon";
    static constexpr uint32_t POLL_MS = 2000u;

    ESP_LOGI(TASK_TAG, "Task started — polling epoch validity every %" PRIu32 " ms", POLL_MS);

    while (true) {
        const bool valid = shelfbot::ShelfbotTimestamp::isEpochValid();
        const time_t now = time(nullptr);
        ESP_LOGI(TASK_TAG, "Current time: %ld (epoch valid? %d)", (long)now, (int)valid);

        const bool currently_synced =
            StateMachine::isInState("time_sync", stateToString(TimeSyncState::SYNCED));

        if (valid && !currently_synced) {
            StateMachine::advance("time_sync");
            if (StateMachine::isInState("time_sync", stateToString(TimeSyncState::SYNCED))) {
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

    EventGroupHandle_t wifi_evt = wifi_manager_get_event_group();
    if (wifi_evt == nullptr) {
        ESP_LOGE(TASK_TAG, "WiFi event group is NULL — wifi event monitor disabled");
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TASK_TAG, "Task started — waiting for first WiFi connection");

    xEventGroupWaitBits(wifi_evt, WM_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TASK_TAG, "Initial WiFi connection observed — disconnect monitor active");

    while (true) {
        xEventGroupWaitBits(wifi_evt, WM_DISCONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
        ESP_LOGW(TASK_TAG, "WiFi disconnected — calling StateMachine::recover()");
        StateMachine::recover();
        xEventGroupWaitBits(wifi_evt, WM_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
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

    while (!StateMachine::isInState("shelfbot", stateToString(ShelfbotState::RUNNING)) &&
           !StateMachine::isInState("shelfbot", stateToString(ShelfbotState::ERROR))) {
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
// create_task — hardened, with arg support and detailed failure logging
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
                 "stack=%u words (%u bytes), priority=%u. "
                 "Free heap: %u bytes, largest free block: %u bytes.",
                 name, (int)ret,
                 (unsigned)stack_words, (unsigned)(stack_words * 4u),
                 (unsigned)priority,
                 (unsigned)esp_get_free_heap_size(),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        return nullptr;
    }

    ESP_LOGI("Shelfbot", "Task created: %-20s  stack=%u words (%u bytes)  priority=%u  handle=%p",
             name,
             (unsigned)stack_words, (unsigned)(stack_words * 4u),
             (unsigned)priority,
             (void*)handle);
    return handle;
}

// Pinned-to-core variant (for WiFi manager — runs alongside WiFi ISRs on core 1)
static TaskHandle_t create_task_pinned(TaskFunction_t  fn,
                                       const char*     name,
                                       uint32_t        stack_words,
                                       UBaseType_t     priority,
                                       void*           arg,
                                       BaseType_t      core_id) {
    TaskHandle_t handle = nullptr;
    BaseType_t ret = xTaskCreatePinnedToCore(fn, name, stack_words, arg,
                                              priority, &handle, core_id);
    if (ret != pdPASS) {
        ESP_LOGE("Shelfbot", "xTaskCreatePinnedToCore('%s' core%d) FAILED (ret=%d) — "
                 "stack=%u words, priority=%u. "
                 "Free heap: %u bytes, largest free block: %u bytes.",
                 name, (int)core_id, (int)ret,
                 (unsigned)stack_words, (unsigned)priority,
                 (unsigned)esp_get_free_heap_size(),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        return nullptr;
    }
    ESP_LOGI("Shelfbot", "Task created: %-20s  stack=%u words  priority=%u  core=%d  handle=%p",
             name, (unsigned)stack_words, (unsigned)priority,
             (int)core_id, (void*)handle);
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
    ESP_LOGI(TAG, "Free heap at boot: %u bytes  largest block: %u bytes",
             (unsigned)esp_get_free_heap_size(),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    ESP_LOGI(TAG, "========================================");

    // ── Initialise StateMachine first – creates mutex and status task ──────
    StateMachine::init();

    // ── Register all modules ───────────────────────────────────────────────
    StateMachine::setInitial("shelfbot",        stateToString(ShelfbotState::SETUP),
                             orderedStates(ShelfbotState()));
    StateMachine::setInitial("led_control",     stateToString(LedControlState::SETUP),
                             orderedStates(LedControlState()));
    StateMachine::setInitial("motor_control",   stateToString(MotorControlState::SETUP),
                             orderedStates(MotorControlState()));
    StateMachine::setInitial("lidar_sensor",    stateToString(LidarSensorState::SETUP),
                             orderedStates(LidarSensorState()));
    StateMachine::setInitial("wifi_manager",    stateToString(WifiManagerState::OFF),
                             orderedStates(WifiManagerState()));
    StateMachine::setInitial("network_service", stateToString(NetworkServiceState::OFF),
                             orderedStates(NetworkServiceState()));
    StateMachine::setInitial("microros_sync",   stateToString(MicrorosState::DISCONNECTED),
                             orderedStates(MicrorosState()));
    StateMachine::setInitial("time_sync",       stateToString(TimeSyncState::UNSYNCED),
                             orderedStates(TimeSyncState()));
    StateMachine::setInitial("agent",           stateToString(AgentState::OFFLINE),
                             orderedStates(AgentState()));

    // ── NVS ───────────────────────────────────────────────────────────────
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

    // ── Hardware components (no tasks yet) ────────────────────────────────
    // These calls initialise driver state and mutexes but do NOT create tasks.
    motor_control_begin();
    led_control_setup();
    lidar_setup();          // creates scan mutex only

    // micro-ROS: registers prerequisites with the state machine.
    MicrorosSync::getInstance().init();

    // ── WiFi hardware init (no task yet) ──────────────────────────────────
    ret = wifi_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "wifi_manager_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // ── Heap snapshot before any task creation ────────────────────────────
    ESP_LOGI(TAG, "Pre-task heap: free=%u bytes  largest_block=%u bytes",
             (unsigned)esp_get_free_heap_size(),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    // ── Create all tasks ──────────────────────────────────────────────────
    //
    // Creation order matters for heap fragmentation: largest stacks first so
    // the allocator places them in the largest available contiguous blocks.
    //
    // microros_task is the largest (48 KB) and must be created first.
    // wifi_mgr is second (12 KB) and is pinned to core 1.
    // All others fit comfortably in remaining space.

    TaskHandle_t microros_handle =
        create_task(MicrorosSync::microros_task_fn,
                    "microros_task",
                    12288u,              // 48 KB — see stack budget table above
                    5u,
                    &MicrorosSync::getInstance());

    TaskHandle_t wifi_mgr_handle =
        create_task_pinned(wifi_manager_task_fn,
                           "wifi_mgr",
                           3072u,        // 12 KB — scan_pick_best + connect frames
                           configMAX_PRIORITIES - 2,
                           nullptr,
                           1);           // core 1: co-located with WiFi ISRs

    TaskHandle_t lidar_lifecycle_handle =
        create_task(lidar_lifecycle_task_fn,
                    "lidar_lifecycle",
                    2560u,               // 10 KB — SM calls + LYDSTO::init
                    3u);

    TaskHandle_t lidar_read_handle =
        create_task(lidar_read_task_fn,
                "lidar_read",
                      2560u,               // 10 KB — same as lidar_lifecycle
               3u);

    TaskHandle_t led_handle =
        create_task(led_lifecycle_task_fn,
                    "led_lifecycle",
                    2048u,               // 8 KB — SM calls only
                    3u);

    TaskHandle_t wifi_monitor_handle =
        create_task(wifi_event_task,
                    "wifi_monitor",
                    2048u,               // 8 KB — event group + SM::recover()
                    4u);

    TaskHandle_t net_services_handle =
        create_task(network_services_task,
                    "net_services",
                    2560u,               // 10 KB — mdns_init + httpd_start
                    5u);

    TaskHandle_t time_sync_handle =
        create_task(time_sync_monitor_task,
                  "time_sync_mon",
                  2048u,
                  5u,   // was 3
                  nullptr);

    TaskHandle_t shelfbot_init_handle =
        create_task(shelfbot_init_task,
                    "shelfbot_init",
                    2560u,               // 10 KB — SM::advance iterates all modules
                    2u);

    // Pass the read task handle to lidar_sensor so lidar_stop() can delete it
    if (lidar_read_handle != nullptr) {
        lidar_set_read_task_handle(lidar_read_handle);
    }

    // ── Verify all critical tasks started ─────────────────────────────────
    bool critical_ok = true;

    if (microros_handle == nullptr) {
        ESP_LOGE(TAG, "CRITICAL: microros_task failed to start");
        critical_ok = false;
    }
    if (wifi_mgr_handle == nullptr) {
        ESP_LOGE(TAG, "CRITICAL: wifi_mgr task failed to start");
        critical_ok = false;
    }
    if (wifi_monitor_handle == nullptr) {
        ESP_LOGE(TAG, "CRITICAL: wifi_monitor task failed to start");
        critical_ok = false;
    }
    if (net_services_handle == nullptr) {
        ESP_LOGE(TAG, "CRITICAL: net_services task failed to start");
        critical_ok = false;
    }
    if (time_sync_handle == nullptr) {
        ESP_LOGE(TAG, "CRITICAL: time_sync_mon task failed to start");
        critical_ok = false;
    }
    if (shelfbot_init_handle == nullptr) {
        ESP_LOGE(TAG, "CRITICAL: shelfbot_init task failed to start");
        critical_ok = false;
    }

    // Non-critical (sensor degraded, not dead)
    if (lidar_lifecycle_handle == nullptr) {
        ESP_LOGE(TAG, "NON-CRITICAL: lidar_lifecycle failed — LiDAR will be unavailable");
    }
    if (lidar_read_handle == nullptr) {
        ESP_LOGE(TAG, "NON-CRITICAL: lidar_read failed — LiDAR will be unavailable");
    }
    if (led_handle == nullptr) {
        ESP_LOGE(TAG, "NON-CRITICAL: led_lifecycle failed — LED will be unavailable");
    }

    if (!critical_ok) {
        ESP_LOGE(TAG, "One or more critical tasks failed to start — setting shelfbot to ERROR. "
                 "Free heap: %u bytes  largest_block: %u bytes",
                 (unsigned)esp_get_free_heap_size(),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        StateMachine::changeState("shelfbot", stateToString(ShelfbotState::ERROR), true);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "All tasks created — post-task heap: free=%u bytes  largest_block=%u bytes",
             (unsigned)esp_get_free_heap_size(),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    ESP_LOGI(TAG, "Components progressing independently via state machine");
    return ESP_OK;
}