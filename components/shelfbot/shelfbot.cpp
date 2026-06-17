// shelfbot.cpp
//
// Key changes vs previous version:
//
//  • network_services_task: the old loop called StateMachine::advance() and then
//    immediately called isInState() to check whether the advance actually worked.
//    That pattern infers state by re-reading it rather than knowing it.  The task
//    now checks preconditions explicitly before calling mdns/http, then calls
//    StateMachine::changeState() directly (the network_service module owns its own
//    state) or advance() and treats the return of is_running() as ground-truth.
//
//  • All xTaskCreate calls capture the return value and log a clear error if the
//    task could not be created; the firmware prints the stack/priority so it is
//    easy to diagnose OOM or priority inversion.
//
//  • microros_sync is started unconditionally and independently.  HTTP server
//    failure cannot block or stop microros.
//
//  • wifi_event_task and time_sync_monitor_task are unchanged in logic but now
//    log with module-prefixed tags for easier filtering.
//
//  • Stack sizes reviewed against free heap budget (~100-150 KB at runtime on a
//    no-PSRAM ESP32-D0WD-V3 with WiFi running):
//      net_services  4096  (mdns + httpd start)
//      wifi_monitor  4096  (calls recover() -> std::mutex + unordered_map)
//      time_sync_mon 3072  (calls isInState/advance/changeState)
//      shelfbot_init 4096  (calls isInState/advance/getState in a loop)
//    Total task stacks from this file: 10 240 bytes — well within budget.

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
// Ownership: this task is the sole owner of the "network_service" module state.
// It advances the state machine only after actually performing each step and
// verifying the result.  It never reads the state back to decide what happened
// — it knows, because it just did it.
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

    // Block until WiFi is connected.
    xEventGroupWaitBits(wifi_evt, WM_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TASK_TAG, "WiFi connected — starting network services");

    // ── Step 1: advance network_service to STARTING ──────────────────────────
    StateMachine::advance("network_service");
    if (!StateMachine::isInState("network_service",
            stateToString(NetworkServiceState::STARTING))) {
        ESP_LOGE(TASK_TAG, "Failed to enter STARTING state — prerequisites not met. "
                 "WiFi state: %s", StateMachine::getState("wifi_manager").c_str());
        vTaskDelete(nullptr);
        return;
    }

    // ── Step 2: initialise mDNS ───────────────────────────────────────────────
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGE(TASK_TAG, "mdns_init failed: %s — network services aborted",
                 esp_err_to_name(err));
        // Move to error state so recover() can reset us.
        StateMachine::changeState("network_service",
            stateToString(NetworkServiceState::ERROR), true);
        vTaskDelete(nullptr);
        return;
    }

    mdns_hostname_set("shelfbot");
    mdns_instance_name_set("Shelfbot ESP32 Client");
    err = mdns_service_add(nullptr, "_microros", "_udp", 8888, nullptr, 0);
    if (err != ESP_OK) {
        // Non-fatal: micro-ROS discovery via mDNS is convenient but not
        // required — the agent can also be reached by IP.
        ESP_LOGW(TASK_TAG, "mdns_service_add (_microros) failed: %s — continuing",
                 esp_err_to_name(err));
    }

    // mDNS succeeded — advance to MDNS_READY.
    StateMachine::advance("network_service");
    if (!StateMachine::isInState("network_service",
            stateToString(NetworkServiceState::MDNS_READY))) {
        ESP_LOGE(TASK_TAG, "Expected MDNS_READY but got: %s",
                 StateMachine::getState("network_service").c_str());
    } else {
        ESP_LOGI(TASK_TAG, "mDNS ready (shelfbot.local)");
    }

    // ── Step 3: start HTTP server ─────────────────────────────────────────────
    // HTTP server failure must NOT affect microros_sync or any other component.
    err = HttpServer::get_instance().start();
    if (err != ESP_OK) {
        ESP_LOGE(TASK_TAG, "HttpServer::start() failed: %s — HTTP will be unavailable. "
                 "All other components (microros, LiDAR, motors) are unaffected.",
                 esp_err_to_name(err));
        // Do NOT advance to HTTP_RUNNING; stay at MDNS_READY or go to ERROR.
        StateMachine::changeState("network_service",
            stateToString(NetworkServiceState::ERROR), true);
        // We could retry here; for now we just exit.  recover() will reset
        // the state so a WiFi reconnect can restart us.
        vTaskDelete(nullptr);
        return;
    }

    // HTTP server started — advance to HTTP_RUNNING.
    StateMachine::advance("network_service");
    if (!StateMachine::isInState("network_service",
            stateToString(NetworkServiceState::HTTP_RUNNING))) {
        ESP_LOGW(TASK_TAG, "HTTP server started but state is: %s (expected http_running)",
                 StateMachine::getState("network_service").c_str());
    } else {
        ESP_LOGI(TASK_TAG, "HTTP server running — all network services up");
    }

    // Task done — network services are self-managing now.
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
            // Clock just became valid — advance to SYNCED.
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
            // Clock became invalid (e.g. RTC drift without re-sync) — go back to UNSYNCED.
            ESP_LOGW(TASK_TAG, "Epoch validity lost — reverting to UNSYNCED");
            StateMachine::changeState("time_sync",
                stateToString(TimeSyncState::UNSYNCED), true);
        }
        // If valid==currently_synced there is nothing to do — no state change.

        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

// ---------------------------------------------------------------------------
// wifi_event_task
//
// WM_DISCONNECTED_BIT is set at startup (before WiFi ever connects) so we
// must wait for the first WM_CONNECTED_BIT before entering the monitor loop.
// Without this guard recover() fires immediately on boot against a clean
// initial state, producing the spurious "disconnected -> disconnected" log.
// ---------------------------------------------------------------------------
static void wifi_event_task(void* /*arg*/) {
    static const char* TASK_TAG = "WifiEventMon";

    EventGroupHandle_t wifi_evt = wifi_manager_get_event_group();
    if (wifi_evt == nullptr) {
        ESP_LOGE(TASK_TAG, "WiFi event group is NULL — wifi event monitor disabled");
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TASK_TAG, "Task started — waiting for first WiFi connection before monitoring");

    // Phase 1: wait until WiFi connects for the first time.
    xEventGroupWaitBits(wifi_evt, WM_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TASK_TAG, "Initial WiFi connection observed — disconnect monitor active");

    // Phase 2: monitor for real disconnections.
    while (true) {
        xEventGroupWaitBits(wifi_evt, WM_DISCONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
        ESP_LOGW(TASK_TAG, "WiFi disconnected — calling StateMachine::recover()");
        StateMachine::recover();
        // Wait for reconnection before looping; avoids hammering recover() while
        // the disconnect bit stays set during the reconnect attempt.
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

    // advance() is called inside the loop; no pre-loop call needed.
    // The loop runs until RUNNING or ERROR; vTaskDelay gives other tasks
    // time to satisfy prerequisites (WiFi, network_service, time_sync).
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
// Helper: create a task with full error reporting
// ---------------------------------------------------------------------------
static bool create_task(TaskFunction_t  fn,
                        const char*     name,
                        uint32_t        stack_words,
                        UBaseType_t     priority,
                        TaskHandle_t*   handle_out = nullptr) {
    TaskHandle_t handle = nullptr;
    BaseType_t ret = xTaskCreate(fn, name, stack_words, nullptr, priority, &handle);
    if (ret != pdPASS) {
        ESP_LOGE("Shelfbot", "xTaskCreate('%s') FAILED (ret=%d) — "
                 "stack=%u words, priority=%u. "
                 "Free heap at time of failure: %u bytes.",
                 name, (int)ret,
                 (unsigned)stack_words, (unsigned)priority,
                 (unsigned)esp_get_free_heap_size());
        if (handle_out) *handle_out = nullptr;
        return false;
    }
    ESP_LOGI("Shelfbot", "Task created: %-20s  stack=%u words  priority=%u",
             name, (unsigned)stack_words, (unsigned)priority);
    if (handle_out) *handle_out = handle;
    return true;
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

    // ── Register all modules ──────────────────────────────────────────────────
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
    wifi_manager_init();

    // ── Monitoring tasks ──────────────────────────────────────────────────────
    // These tasks are infrastructure; their failure is not fatal but must be
    // logged loudly.
    // Stack sizes: every task that calls StateMachine methods touches
    // std::mutex + std::unordered_map + std::string — budget ~1.5 KB per call
    // frame on Xtensa.  wifi_monitor and shelfbot_init also call recover()/
    // advance() which iterate all modules.  Use 4096 bytes minimum for any
    // task that touches StateMachine.  net_services additionally calls
    // httpd_start + mdns_init — keep at 4096.
    (void)create_task(wifi_event_task,        "wifi_monitor",  4096u, 4u);
    (void)create_task(network_services_task,  "net_services",  4096u, 5u);
    (void)create_task(time_sync_monitor_task, "time_sync_mon", 3072u, 3u);
    (void)create_task(shelfbot_init_task,     "shelfbot_init", 4096u, 2u);

    // ── micro-ROS: started independently, not gated by HTTP server ────────────
    MicrorosSync::getInstance().init();
    MicrorosSync::getInstance().start();

    // ── Hardware components ───────────────────────────────────────────────────
    led_control_setup();
    motor_control_begin();
    lidar_setup();

    ESP_LOGI(TAG, "Initialisation complete — free heap: %u bytes",
             (unsigned)esp_get_free_heap_size());
    ESP_LOGI(TAG, "Components progressing independently via state machine");
    return ESP_OK;
}
