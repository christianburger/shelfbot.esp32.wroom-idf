// shelfbot.cpp
//
// ALL FreeRTOS task creation happens here — no other component spawns tasks.
// This gives one auditable location for the entire heap/stack budget.
//
// Task inventory (in creation order, largest stack first):
//
//  Name              Stack (words)  Stack (bytes)  Notes
//  ─────────────────────────────────────────────────────────────────────────
//  microros_task     12288          49152           XRCE session context
//  sm_dump            4096          16384           SM status every 10 s
//  time_sync_mon      4096          16384           Calls SM::advance(); needs headroom
//  wifi_mgr           3072          12288           Pinned core 1; scan_pick_best on stack
//  lidar_lifecycle    2560          10240           One-shot; SM calls + LYDSTO init
//  lidar_read         2560          10240           Tight UART loop
//  net_services       2560          10240           mdns_init + httpd_start
//  shelfbot_init      2560          10240           SM::advance iterates all modules
//  led_lifecycle      2048           8192           SM calls only
//  wifi_monitor       2048           8192           Event group + SM::recover()
//
// Creation order matters for heap fragmentation: largest stacks are
// allocated first when the heap is most contiguous.

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
//
// One-shot task: sets up mDNS and starts the HTTP server after WiFi connects.
//
// Retry model for state machine transitions
// -----------------------------------------
// OFF → STARTING requires wifi_manager >= CONNECTED.  We wait for
// WM_CONNECTED_BIT (event group) before even calling advance(), so the
// prerequisite should be satisfied immediately.  However, because the event
// bit is set in the IP event handler just before set_wifi_state(), there is a
// tiny race window.  Rather than give up on the first failed advance() call
// (which caused a permanent loss of network services), we retry up to 20
// times (10 s total) so the race resolves naturally.
//
// STARTING → MDNS_READY and MDNS_READY → HTTP_RUNNING have no prerequisites;
// they are advanced inline after each service starts.
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

    // ── OFF → STARTING ────────────────────────────────────────────────────
    // Prerequisite: wifi_manager >= CONNECTED.  Retry until satisfied.
    // (The event bit guarantees WiFi is up; retries handle the race window.)
    {
        int attempts = 0;
        while (!StateMachine::isInState("network_service",
                                         stateToString(NetworkServiceState::STARTING))) {
            if (StateMachine::advance("network_service")) break;
            ++attempts;
            if (attempts >= 20) {
                ESP_LOGE(TASK_TAG,
                    "Cannot enter STARTING after %d attempts "
                    "(wifi=%s) — aborting network services",
                    attempts, StateMachine::getState("wifi_manager").c_str());
                vTaskDelete(nullptr);
                return;
            }
            ESP_LOGW(TASK_TAG,
                "advance to STARTING failed (attempt %d, wifi=%s) — retrying in 500 ms",
                attempts, StateMachine::getState("wifi_manager").c_str());
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    // ── mDNS ──────────────────────────────────────────────────────────────
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

    // ── STARTING → MDNS_READY ─────────────────────────────────────────────
    // No prerequisites; advance always succeeds if state is STARTING.
    StateMachine::advance("network_service");
    if (!StateMachine::isInState("network_service",
            stateToString(NetworkServiceState::MDNS_READY))) {
        ESP_LOGE(TASK_TAG, "Expected MDNS_READY but got: %s",
                 StateMachine::getState("network_service").c_str());
    } else {
        ESP_LOGI(TASK_TAG, "mDNS ready (shelfbot.local)");
    }

    // ── HTTP server ────────────────────────────────────────────────────────
    err = HttpServer::get_instance().start();
    if (err != ESP_OK) {
        ESP_LOGE(TASK_TAG, "HttpServer::start() failed: %s — HTTP will be unavailable.",
                 esp_err_to_name(err));
        StateMachine::changeState("network_service",
            stateToString(NetworkServiceState::ERROR), true);
        vTaskDelete(nullptr);
        return;
    }

    // ── MDNS_READY → HTTP_RUNNING ─────────────────────────────────────────
    StateMachine::advance("network_service");
    if (!StateMachine::isInState("network_service",
            stateToString(NetworkServiceState::HTTP_RUNNING))) {
        ESP_LOGW(TASK_TAG, "HTTP server started but state is: %s (expected http_running)",
                 StateMachine::getState("network_service").c_str());
    } else {
        ESP_LOGI(TASK_TAG, "HTTP server running — all network services up");
    }

    // One-shot task is complete.
    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// time_sync_monitor_task
//
// Polls wall-clock validity every 2 s.  Advances time_sync UNSYNCED→SYNCED
// when the epoch becomes valid.  Reverts to UNSYNCED if the clock is lost.
//
// Advance retry: if advance() returns false (e.g. wifi or network not yet
// ready when the epoch first becomes valid), the next poll iteration (2 s
// later) will call it again.  No extra retry logic is needed.
//
// Stack: 4096 words (16 KB).  StateMachine::advance() constructs a local
// std::unordered_map + std::vector<std::string> and calls getOrderedStates()
// which returns a std::vector by value.  These temporaries overflow a 2048-
// word (8 KB) stack — verified by the crash in the field.
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
            // Prerequisites: wifi_manager >= CONNECTED, network_service >= MDNS_READY.
            // If not yet met, advance() returns false and the next poll retries.
            if (StateMachine::advance("time_sync")) {
                ESP_LOGI(TASK_TAG, "Time sync achieved — epoch is valid");
            } else {
                ESP_LOGW(TASK_TAG,
                    "Epoch valid but advance blocked "
                    "(wifi=%s network=%s) — will retry in %" PRIu32 " ms",
                    StateMachine::getState("wifi_manager").c_str(),
                    StateMachine::getState("network_service").c_str(),
                    POLL_MS);
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
//
// Advances the top-level "shelfbot" module through SETUP → INIT → RUNNING.
//
//  SETUP → INIT:    no prerequisites, succeeds immediately.
//  INIT  → RUNNING: requires wifi >= CONNECTED, network >= MDNS_READY,
//                   time_sync >= SYNCED.
//
// The task retries every 1 s until RUNNING (or ERROR) is reached.
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
// create_task — hardened wrapper with heap diagnostics
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

// Pinned-to-core variant (WiFi manager — runs alongside WiFi ISRs on core 1)
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

    // ── StateMachine: mutex + task_running flag only (no task spawned here) ──
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

    // ── Hardware components (mutex/state init only — no tasks) ────────────
    motor_control_begin();
    led_control_setup();
    lidar_setup();              // creates scan mutex only

    // micro-ROS: registers state machine prerequisites.
    MicrorosSync::getInstance().init();

    // ── WiFi hardware init (event handlers only — no task) ────────────────
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
    // ORDER: largest stacks first so the allocator places them in the largest
    // available contiguous blocks before fragmentation sets in.
    //
    // ── Tier 1: ≥ 12 KB ───────────────────────────────────────────────────

    // micro-ROS executor + XRCE session (48 KB).
    // arg = singleton instance so the static task function can reach 'this'.
    TaskHandle_t microros_handle =
        create_task(MicrorosSync::microros_task_fn,
                    "microros_task",
                    12288u,
                    5u,
                    &MicrorosSync::getInstance());

    // ── Tier 2: 4 KB (16 KB each) ─────────────────────────────────────────

    // StateMachine periodic state dump.
    // Must be created AFTER StateMachine::init() sets task_running_ = true.
    // Stack: 4096 words — dumpAllStates() iterates std::unordered_map with
    // std::string keys and calls ESP_LOGI inside the mutex.
    TaskHandle_t sm_dump_handle =
        create_task(StateMachine::status_dump_task_fn,
                    "sm_dump",
                    4096u,
                    tskIDLE_PRIORITY + 1,
                    nullptr);

    // Time-sync monitor.
    // Stack: 4096 words — StateMachine::advance() constructs local
    // std::unordered_map + std::vector<std::string> for all modules.
    // 2048 words (8 KB) caused a confirmed stack overflow in the field.
    TaskHandle_t time_sync_handle =
        create_task(time_sync_monitor_task,
                    "time_sync_mon",
                    4096u,
                    5u,
                    nullptr);

    // ── Tier 3: 3 KB (12 KB) ──────────────────────────────────────────────

    // WiFi manager — pinned to core 1 (co-located with WiFi ISRs).
    // scan_pick_best() keeps up to SCAN_MAX_APS=20 wifi_ap_record_t on stack.

    TaskHandle_t wifi_mgr_handle =
      create_task_pinned(WifiManager::task_fn,     // ← use static task_fn
                       "wifi_mgr",
                       4096u,
                       configMAX_PRIORITIES - 2,
                       &WifiManager::getInstance(), // ← pass singleton pointer
                       1);

    WifiManager::getInstance().setTaskHandle(wifi_mgr_handle);

    // ── Tier 4: 2.5 KB (10 KB each) ───────────────────────────────────────

    // LiDAR lifecycle (one-shot init task — terminates via vTaskDelete(nullptr)).
    TaskHandle_t lidar_lifecycle_handle =
        create_task(lidar_lifecycle_task_fn,
                    "lidar_lifecycle",
                    2560u,
                    3u);

    // LiDAR read loop (permanent — exits only when s_stop_requested is set).
    TaskHandle_t lidar_read_handle =
        create_task(lidar_read_task_fn,
                    "lidar_read",
                    2560u,
                    3u);

    // Network services (one-shot — terminates via vTaskDelete(nullptr)).
    TaskHandle_t net_services_handle =
        create_task(network_services_task,
                    "net_services",
                    2560u,
                    5u);

    // Shelfbot top-level init (one-shot — terminates via vTaskDelete(nullptr)).
    TaskHandle_t shelfbot_init_handle =
        create_task(shelfbot_init_task,
                    "shelfbot_init",
                    2560u,
                    2u);

    // ── Tier 5: 2 KB (8 KB each) ──────────────────────────────────────────

    // LED lifecycle (permanent idle loop after reaching RUNNING).
    TaskHandle_t led_handle =
        create_task(led_lifecycle_task_fn,
                    "led_lifecycle",
                    2048u,
                    3u);

    // WiFi disconnect monitor (permanent event loop).
    TaskHandle_t wifi_monitor_handle =
        create_task(wifi_event_task,
                    "wifi_monitor",
                    2048u,
                    4u);

    // ── Pass read-task handle to lidar_sensor for clean-shutdown support ──
    if (lidar_read_handle != nullptr) {
        lidar_set_read_task_handle(lidar_read_handle);
    }

    // ── Verify critical tasks ─────────────────────────────────────────────
    bool critical_ok = true;

#define CHECK_CRITICAL(handle, label)                                       \
    if ((handle) == nullptr) {                                              \
        ESP_LOGE(TAG, "CRITICAL: " label " task failed to start");          \
        critical_ok = false;                                                \
    }

    CHECK_CRITICAL(microros_handle,     "microros_task")
    CHECK_CRITICAL(sm_dump_handle,      "sm_dump")
    CHECK_CRITICAL(time_sync_handle,    "time_sync_mon")
    CHECK_CRITICAL(wifi_mgr_handle,     "wifi_mgr")
    CHECK_CRITICAL(wifi_monitor_handle, "wifi_monitor")
    CHECK_CRITICAL(net_services_handle, "net_services")
    CHECK_CRITICAL(shelfbot_init_handle,"shelfbot_init")

#undef CHECK_CRITICAL

    // Non-critical: sensor degraded, not dead.
    if (lidar_lifecycle_handle == nullptr)
        ESP_LOGE(TAG, "NON-CRITICAL: lidar_lifecycle failed — LiDAR will be unavailable");
    if (lidar_read_handle == nullptr)
        ESP_LOGE(TAG, "NON-CRITICAL: lidar_read failed — LiDAR will be unavailable");
    if (led_handle == nullptr)
        ESP_LOGE(TAG, "NON-CRITICAL: led_lifecycle failed — LED will be unavailable");

    if (!critical_ok) {
        ESP_LOGE(TAG,
            "One or more critical tasks failed to start — setting shelfbot to ERROR. "
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
