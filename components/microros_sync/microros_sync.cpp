#include "microros_sync.hpp"
#include <wifi_manager.hpp>
#include <firmware_version.hpp>
#include <state_machine.hpp>
#include <state_machine_lifecycle.hpp>
#include <rmw_microros.h>
#include <esp_log.h>
#include <cstring>
#include <lidar_sensor.hpp>

static const char* TAG = "MicrorosSync";
MicrorosSync* MicrorosSync::instance_ = nullptr;
SemaphoreHandle_t MicrorosSync::mutex_ = nullptr;

bool lockCore() {
    return MicrorosSync::lockCore();
}
void unlockCore() {
    MicrorosSync::unlockCore();
}

MicrorosSync& MicrorosSync::getInstance() {
    static MicrorosSync instance;
    return instance;
}

MicrorosSync::MicrorosSync()
    : node_(rcl_get_zero_initialized_node()),
      allocator_(rcl_get_default_allocator()),
      support_{},
      executor_(rclc_executor_get_zero_initialized_executor()),
      entities_created_(false),
      support_inited_(false)
{
    instance_ = this;
    if (mutex_ == nullptr) {
        mutex_ = xSemaphoreCreateMutex();
        configASSERT(mutex_ != nullptr);
    }
    for (int i = 0; i < COMP_COUNT; ++i) comp_initialized_[i] = false;
    std_msgs__msg__Int32__init(&heartbeat_msg_);
    heartbeat_pub_   = rcl_get_zero_initialized_publisher();
    heartbeat_timer_ = rcl_get_zero_initialized_timer();
    lidar_timer_     = rcl_get_zero_initialized_timer();
}

MicrorosSync::~MicrorosSync() {
    // Task is deleted externally; do not delete it here.
    instance_ = nullptr;
}

// ─── Lock helpers (defensive) ──────────────────────────────────────────────
bool MicrorosSync::lockCore() {
    if (mutex_ == nullptr) {
        ESP_LOGE(TAG, "lockCore: mutex is NULL – cannot lock!");
        return false;
    }
    // The mutex is a binary semaphore; xSemaphoreTake is safe.
    return xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) == pdTRUE;
}

void MicrorosSync::unlockCore() {
    if (mutex_ != nullptr) {
        xSemaphoreGive(mutex_);
    }
}

bool MicrorosSync::init() {
    FirmwareVersion::print_version(TAG);
    StateMachine::registerPrerequisite("microros_sync", stateToString(MicrorosState::DISCOVERING),
        { "wifi_manager", stateToString(WifiManagerState::CONNECTED) });
    StateMachine::registerPrerequisite("microros_sync", stateToString(MicrorosState::DISCOVERING),
        { "network_service", stateToString(NetworkServiceState::MDNS_READY) });
    StateMachine::registerPrerequisite("microros_sync", stateToString(MicrorosState::CREATING_ENTITIES),
        { "time_sync", stateToString(TimeSyncState::SYNCED) });
    return true;
}

void MicrorosSync::start() {
    // Task is now created externally (in Shelfbot::begin()).
    ESP_LOGI(TAG, "micro-ROS sync ready; task created externally");
}

bool MicrorosSync::isConnected() const {
    return StateMachine::isInState("microros_sync", stateToString(MicrorosState::CONNECTED));
}

bool MicrorosSync::queryAgentIp(char* out_ip, size_t len) {
    esp_ip4_addr_t addr = { .addr = 0 };
    esp_err_t err = mdns_query_a(CONFIG_MICROROS_AGENT_MDNS_HOST, 2000, &addr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mDNS query for %s failed: %s", CONFIG_MICROROS_AGENT_MDNS_HOST, esp_err_to_name(err));
        return false;
    }
    snprintf(out_ip, len, IPSTR, IP2STR(&addr));
    ESP_LOGI(TAG, "mDNS resolved %s -> %s", CONFIG_MICROROS_AGENT_MDNS_HOST, out_ip);
    return true;
}

void MicrorosSync::logMicrorosLimits() {
    ESP_LOGI(TAG, "=== micro-ROS compile-time limits ===");
    ESP_LOGI(TAG, "RMW_UXRCE_MAX_NODES         = %d", RMW_UXRCE_MAX_NODES);
    ESP_LOGI(TAG, "RMW_UXRCE_MAX_PUBLISHERS    = %d", RMW_UXRCE_MAX_PUBLISHERS);
    ESP_LOGI(TAG, "RMW_UXRCE_MAX_SUBSCRIPTIONS = %d", RMW_UXRCE_MAX_SUBSCRIPTIONS);
    ESP_LOGI(TAG, "RMW_UXRCE_MAX_SERVICES      = %d", RMW_UXRCE_MAX_SERVICES);
    ESP_LOGI(TAG, "RMW_UXRCE_MAX_CLIENTS       = %d", RMW_UXRCE_MAX_CLIENTS);
    ESP_LOGI(TAG, "RMW_UXRCE_MAX_HISTORY       = %d", RMW_UXRCE_MAX_HISTORY);
    ESP_LOGI(TAG, "=====================================");
}

bool MicrorosSync::createEntitiesImpl() {
    for (int i = 0; i < COMP_COUNT; ++i) comp_initialized_[i] = false;

    node_            = rcl_get_zero_initialized_node();
    executor_        = rclc_executor_get_zero_initialized_executor();
    heartbeat_pub_   = rcl_get_zero_initialized_publisher();
    heartbeat_timer_ = rcl_get_zero_initialized_timer();
    lidar_timer_     = rcl_get_zero_initialized_timer();

    rcl_ret_t r;
    r = rclc_node_init_default(&node_, "shelfbot_firmware", "", &support_);
    if (r != RCL_RET_OK) {
        ESP_LOGE(TAG, "node init failed: %ld", (long)r);
        return false;
    }

    r = rclc_executor_init(&executor_, &support_.context, 10, &allocator_);
    if (r != RCL_RET_OK) {
        ESP_LOGE(TAG, "executor init failed: %ld", (long)r);
        return false;
    }

    logMicrorosLimits();

    if (!led_.init(&node_, &executor_)) return false;
    comp_initialized_[COMP_LED] = true;

    if (!motors_.init(&node_, &support_, &executor_)) return false;
    comp_initialized_[COMP_MOTORS] = true;

    if (!lidar_.init(&node_, &executor_)) return false;
    comp_initialized_[COMP_LIDAR] = true;

    r = rclc_publisher_init_best_effort(&heartbeat_pub_, &node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "shelfbot_firmware/heartbeat");
    if (r != RCL_RET_OK) {
        ESP_LOGE(TAG, "heartbeat publisher init failed: %ld", (long)r);
        return false;
    }
    r = rclc_timer_init_default(&heartbeat_timer_, &support_, RCL_MS_TO_NS(1000),
                                 heartbeatTimerCallback);
    if (r != RCL_RET_OK) {
        ESP_LOGE(TAG, "heartbeat timer init failed: %ld", (long)r);
        return false;
    }
    r = rclc_executor_add_timer(&executor_, &heartbeat_timer_);
    if (r != RCL_RET_OK) {
        ESP_LOGE(TAG, "add heartbeat timer failed: %ld", (long)r);
        return false;
    }

    r = rclc_timer_init_default(&lidar_timer_, &support_, RCL_MS_TO_NS(200),
                                 lidarTimerCallback);
    if (r != RCL_RET_OK) {
        ESP_LOGE(TAG, "lidar timer init failed: %ld", (long)r);
        return false;
    }
    r = rclc_executor_add_timer(&executor_, &lidar_timer_);
    if (r != RCL_RET_OK) {
        ESP_LOGE(TAG, "add lidar timer failed: %ld", (long)r);
        return false;
    }

    entities_created_ = true;
    ESP_LOGI(TAG, "micro-ROS entities created");
    return true;
}

void MicrorosSync::destroyEntitiesImpl() {
    if (!entities_created_) return;

    if (comp_initialized_[COMP_LIDAR])      lidar_.fini(&node_);
    if (comp_initialized_[COMP_MOTORS])     motors_.fini(&node_);
    if (comp_initialized_[COMP_LED])        led_.fini(&node_);

    rcl_ret_t r;
    r = rcl_publisher_fini(&heartbeat_pub_, &node_);
    if (r != RCL_RET_OK) ESP_LOGW(TAG, "heartbeat_pub fini: %ld", (long)r);
    r = rcl_timer_fini(&heartbeat_timer_);
    if (r != RCL_RET_OK) ESP_LOGW(TAG, "heartbeat_timer fini: %ld", (long)r);
    r = rcl_timer_fini(&lidar_timer_);
    if (r != RCL_RET_OK) ESP_LOGW(TAG, "lidar_timer fini: %ld", (long)r);
    r = rclc_executor_fini(&executor_);
    if (r != RCL_RET_OK) ESP_LOGW(TAG, "executor fini: %ld", (long)r);
    r = rcl_node_fini(&node_);
    if (r != RCL_RET_OK) ESP_LOGW(TAG, "node fini: %ld", (long)r);

    node_            = rcl_get_zero_initialized_node();
    executor_        = rclc_executor_get_zero_initialized_executor();
    heartbeat_pub_   = rcl_get_zero_initialized_publisher();
    heartbeat_timer_ = rcl_get_zero_initialized_timer();
    lidar_timer_     = rcl_get_zero_initialized_timer();

    if (support_inited_) {
        r = rclc_support_fini(&support_);
        if (r != RCL_RET_OK) ESP_LOGW(TAG, "support_fini failed: %ld", (long)r);
        memset(&support_, 0, sizeof(support_));
        support_inited_ = false;
    }

    for (int i = 0; i < COMP_COUNT; ++i) comp_initialized_[i] = false;
    entities_created_ = false;
    ESP_LOGI(TAG, "micro-ROS entities destroyed");
}

void MicrorosSync::heartbeatTimerCallback(rcl_timer_t*, int64_t) {
    if (!instance_ || !instance_->isConnected()) return;
    static int32_t counter = 0;
    instance_->heartbeat_msg_.data = ++counter;
    publish_or_fail(&instance_->heartbeat_pub_, &instance_->heartbeat_msg_, "heartbeat");
}

// ─── LiDAR timer callback ──────────────────────────────────────────────────
void MicrorosSync::lidarTimerCallback(rcl_timer_t*, int64_t) {
    if (!instance_ || !instance_->isConnected()) return;

    static uint32_t s_not_running_log = 0;
    if (!lidar_is_running()) {
        if ((++s_not_running_log % 25) == 0) {
            ESP_LOGW("MicrorosSync", "lidarTimer: lidar not running yet (check=%lu)", 
                     (unsigned long)s_not_running_log);
        }
        return;
    }

    static LidarScan scan;
    static uint32_t s_no_scan_log = 0;

    if (lidar_get_latest_scan(scan)) {
        ESP_LOGD("MicrorosSync", "lidarTimer: got scan pts=%u, publishing...", 
                 (unsigned)scan.point_count);
        s_no_scan_log = 0;
        instance_->lidar_.publishLidarScan(scan);
    } else {
        if ((++s_no_scan_log % 25) == 0) {
            ESP_LOGD("MicrorosSync", "lidarTimer: no scan ready yet (polls=%lu, scan_count=%lu)",
                     (unsigned long)s_no_scan_log,
                     (unsigned long)lidar_get_scan_count());
        }
    }
}

// ─── Task function (now public) ────────────────────────────────────────────
void MicrorosSync::microros_task(void* arg) {
    MicrorosSync* self = static_cast<MicrorosSync*>(arg);
    char agent_ip[16]  = {};

    while (true) {
        std::string current_state = StateMachine::getState("microros_sync");

        if (current_state == stateToString(MicrorosState::ERROR) ||
            current_state == stateToString(MicrorosState::RECOVERING)) {
            StateMachine::recover();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (current_state == stateToString(MicrorosState::DISCONNECTED)) {
            StateMachine::advance("microros_sync");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (current_state == stateToString(MicrorosState::DISCOVERING)) {
            if (!StateMachine::isInState("agent", stateToString(AgentState::DISCOVERED))) {
                if (!self->queryAgentIp(agent_ip, sizeof(agent_ip))) {
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    continue;
                }
                StateMachine::changeState("agent", stateToString(AgentState::DISCOVERED), true);
            }

            if (!StateMachine::isAtLeast("time_sync", stateToString(TimeSyncState::SYNCED))) {
                ESP_LOGI(TAG, "Waiting for time sync before connecting to agent...");
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            if (!self->support_inited_) {
                rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
                rcl_ret_t r = rcl_init_options_init(&init_options, self->allocator_);
                if (r != RCL_RET_OK) {
                    ESP_LOGE(TAG, "init_options_init failed: %ld", (long)r);
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    continue;
                }

                rmw_ret_t rmw_r = rmw_uros_options_set_udp_address(agent_ip, "8888",
                    rcl_init_options_get_rmw_init_options(&init_options));
                if (rmw_r != RMW_RET_OK) {
                    ESP_LOGE(TAG, "rmw_uros_options_set_udp_address failed: %ld", (long)rmw_r);
                    rcl_ret_t fini_r = rcl_init_options_fini(&init_options);
                    if (fini_r != RCL_RET_OK)
                        ESP_LOGW(TAG, "rcl_init_options_fini after set_udp_address error: %ld", (long)fini_r);
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    continue;
                }
                ESP_LOGI(TAG, "Agent address set to %s:8888", agent_ip);

                r = rclc_support_init_with_options(&self->support_, 0, nullptr,
                                                   &init_options, &self->allocator_);
                rcl_ret_t fini_r = rcl_init_options_fini(&init_options);
                if (fini_r != RCL_RET_OK)
                    ESP_LOGW(TAG, "rcl_init_options_fini after support_init: %ld", (long)fini_r);
                if (r != RCL_RET_OK) {
                    ESP_LOGE(TAG, "support_init failed: %ld", (long)r);
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    continue;
                }
                self->support_inited_ = true;
                ESP_LOGI(TAG, "Transport initialized to %s:8888", agent_ip);
            }

            if (!StateMachine::isInState("agent", stateToString(AgentState::PING_OK))) {
                ESP_LOGI(TAG, "Pinging agent at %s...", agent_ip);
                constexpr int PING_TIMEOUT_MS = 2000;
                constexpr int PING_ATTEMPTS   = 3;
                if (rmw_uros_ping_agent(PING_TIMEOUT_MS, PING_ATTEMPTS) == RMW_RET_OK) {
                    ESP_LOGI(TAG, "Agent ping OK");
                    StateMachine::changeState("agent", stateToString(AgentState::PING_OK), true);
                } else {
                    ESP_LOGW(TAG, "Agent ping FAILED (timeout=%dms attempts=%d) - destroying transport and retrying",
                             PING_TIMEOUT_MS, PING_ATTEMPTS);
                    (void)rclc_support_fini(&self->support_);
                    self->support_inited_ = false;
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    continue;
                }
            }

            if (!StateMachine::isInState("agent", stateToString(AgentState::SESSION_SYNCED))) {
                if (rmw_uros_sync_session(5000) != RMW_RET_OK) {
                    ESP_LOGE(TAG, "Session sync failed");
                    (void)rclc_support_fini(&self->support_);
                    self->support_inited_ = false;
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    continue;
                }
                StateMachine::changeState("agent", stateToString(AgentState::SESSION_SYNCED), true);
            }

            StateMachine::advance("microros_sync");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (current_state == stateToString(MicrorosState::CREATING_ENTITIES)) {
            if (!self->createEntitiesImpl()) {
                ESP_LOGE(TAG, "Entity creation failed, cleaning up and retrying...");
                self->destroyEntitiesImpl();
                if (self->support_inited_) {
                    (void)rclc_support_fini(&self->support_);
                    self->support_inited_ = false;
                }
                memset(agent_ip, 0, sizeof(agent_ip));
                StateMachine::recover();
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            StateMachine::advance("microros_sync");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (current_state == stateToString(MicrorosState::CONNECTED)) {
            static TickType_t last_ping_tick = 0;
            constexpr uint32_t PING_INTERVAL_MS = 2000;
            constexpr int      PING_TIMEOUT_MS  = 500;
            constexpr int      PING_ATTEMPTS    = 2;

            TickType_t now = xTaskGetTickCount();
            if ((now - last_ping_tick) >= pdMS_TO_TICKS(PING_INTERVAL_MS)) {
                last_ping_tick = now;
                if (rmw_uros_ping_agent(PING_TIMEOUT_MS, PING_ATTEMPTS) != RMW_RET_OK) {
                    ESP_LOGW(TAG, "Agent keepalive ping failed - connection lost");
                    StateMachine::recover();
                    continue;
                }
                ESP_LOGD(TAG, "Agent keepalive OK");
            }

            rcl_ret_t spin_r = rclc_executor_spin_some(&self->executor_, RCL_MS_TO_NS(100));
            if (spin_r != RCL_RET_OK && spin_r != RCL_RET_TIMEOUT) {
                ESP_LOGW(TAG, "spin_some error: %ld", (long)spin_r);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        StateMachine::recover();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ── Static publish helpers ────────────────────────────────────────────────────
void MicrorosSync::publishHeartbeat(int32_t value) {
    if (!instance_ || !instance_->isConnected()) return;
    instance_->heartbeat_msg_.data = value;
    publish_or_fail(&instance_->heartbeat_pub_, &instance_->heartbeat_msg_, "heartbeat");
}

void MicrorosSync::publishMotorPositions(const float* positions, size_t count) {
    if (!instance_ || !instance_->isConnected()) return;
    instance_->motors_.publishPositions(positions, count);
}

void MicrorosSync::publishLedState(bool state) {
    if (!instance_ || !instance_->isConnected()) return;
    instance_->led_.publishState(state);
}

void MicrorosSync::publishLidarScan(const LidarScan& scan) {
    if (!instance_ || !instance_->isConnected()) return;
    instance_->lidar_.publishLidarScan(scan);
}