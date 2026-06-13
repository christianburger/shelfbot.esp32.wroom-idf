#include "state_machine.hpp"
#include "state_machine_lifecycle.hpp"
#include <esp_timer.h>
#include <esp_log.h>

static const char* TAG = "StateMachine";

// Static member definitions
std::mutex StateMachine::mutex_;
std::unordered_map<std::string, StateMachine::ModuleState> StateMachine::modules_;
std::unordered_map<
    std::string,
    std::unordered_map<std::string, std::vector<StateMachine::Prerequisite>>
> StateMachine::prerequisites_;
TaskHandle_t StateMachine::status_task_handle_ = nullptr;
bool         StateMachine::task_running_       = false;

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------
void StateMachine::init() {
    if (task_running_) {
        ESP_LOGW(TAG, "Already initialised");
        return;
    }
    task_running_ = true;
    const BaseType_t ret = xTaskCreate(
        status_dump_task, "sm_dump", 4096, nullptr,
        tskIDLE_PRIORITY + 1, &status_task_handle_);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create status dump task");
        task_running_ = false;
    } else {
        ESP_LOGI(TAG, "Status dump task started");
        ESP_LOGI(TAG, "StateMachine initialised, status logged every 10 seconds");
    }
}

// ---------------------------------------------------------------------------
// setInitial
// ---------------------------------------------------------------------------
bool StateMachine::setInitial(const std::string& module,
                              const std::string& initial_state,
                              const std::vector<std::string>& ordered_states) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (modules_.find(module) != modules_.end()) {
        ESP_LOGW(TAG, "Module '%s' already registered (ignored)", module.c_str());
        return false;
    }
    if (!ordered_states.empty()) {
        bool found = false;
        for (const auto& s : ordered_states) {
            if (s == initial_state) { found = true; break; }
        }
        if (!found) {
            ESP_LOGE(TAG, "Module '%s': initial_state '%s' not in ordered_states",
                     module.c_str(), initial_state.c_str());
        }
    }
    modules_.emplace(module, ModuleState(initial_state, ordered_states));
    ESP_LOGI(TAG, "Module '%s' initial state: %s", module.c_str(), initial_state.c_str());
    return true;
}

// ---------------------------------------------------------------------------
// registerPrerequisite
// ---------------------------------------------------------------------------
void StateMachine::registerPrerequisite(const std::string& module,
                                         const std::string& target_state,
                                         const Prerequisite& prereq) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (modules_.find(module) == modules_.end())
        ESP_LOGE(TAG, "registerPrerequisite: subject module '%s' not yet registered", module.c_str());
    if (modules_.find(prereq.prereq_module) == modules_.end())
        ESP_LOGE(TAG, "registerPrerequisite: prereq module '%s' not yet registered", prereq.prereq_module.c_str());

    prerequisites_[module][target_state].push_back(prereq);
    ESP_LOGI(TAG, "Prereq registered: '%s' -> '%s' requires '%s' >= '%s'",
             module.c_str(), target_state.c_str(),
             prereq.prereq_module.c_str(), prereq.min_state.c_str());
}

// ---------------------------------------------------------------------------
// prereqsSatisfied_locked (mutex held)
// ---------------------------------------------------------------------------
bool StateMachine::prereqsSatisfied_locked(const std::string& module,
                                           const std::string& new_state) {
    auto mod_it = prerequisites_.find(module);
    if (mod_it == prerequisites_.end()) return true;

    auto state_it = mod_it->second.find(new_state);
    if (state_it == mod_it->second.end()) return true;

    for (const auto& prereq : state_it->second) {
        auto pmod_it = modules_.find(prereq.prereq_module);
        if (pmod_it == modules_.end()) {
            ESP_LOGW(TAG, "Prereq module '%s' not registered (blocking '%s' -> '%s')",
                     prereq.prereq_module.c_str(), module.c_str(), new_state.c_str());
            return false;
        }
        const ModuleState& ps = pmod_it->second;
        const int cur_rank  = ps.rank(ps.current_state);
        const int need_rank = ps.rank(prereq.min_state);

        if (need_rank < 0) {
            ESP_LOGE(TAG, "Prereq min_state '%s' has no rank in module '%s'",
                     prereq.min_state.c_str(), prereq.prereq_module.c_str());
            return false;
        }
        if (cur_rank < need_rank) {
            ESP_LOGD(TAG, "'%s' -> '%s' blocked: '%s' is '%s' (rank %d), needs '%s' (rank %d)",
                     module.c_str(), new_state.c_str(),
                     prereq.prereq_module.c_str(), ps.current_state.c_str(), cur_rank,
                     prereq.min_state.c_str(), need_rank);
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// canTransition
// ---------------------------------------------------------------------------
bool StateMachine::canTransition(const std::string& module,
                                  const std::string& new_state) {
    std::lock_guard<std::mutex> lock(mutex_);
    return prereqsSatisfied_locked(module, new_state);
}

// ---------------------------------------------------------------------------
// waitForPrerequisites
// ---------------------------------------------------------------------------
bool StateMachine::waitForPrerequisites(const std::string& module,
                                         const std::string& target_state,
                                         uint32_t timeout_ms,
                                         uint32_t poll_ms) {
    const TickType_t start   = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(timeout_ms);
    const TickType_t poll    = pdMS_TO_TICKS(poll_ms);

    while (true) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (prereqsSatisfied_locked(module, target_state)) return true;
        }
        if ((xTaskGetTickCount() - start) >= timeout) {
            ESP_LOGW(TAG, "Timeout waiting for '%s' prerequisites to enter '%s'",
                     module.c_str(), target_state.c_str());
            return false;
        }
        vTaskDelay(poll);
    }
}

// ---------------------------------------------------------------------------
// changeState (private – only called by advance() and recover())
// ---------------------------------------------------------------------------
bool StateMachine::changeState(const std::string& module,
                               const std::string& new_state,
                               bool force_skip_prereqs) {
    auto it = modules_.find(module);
    if (it == modules_.end()) {
        ESP_LOGE(TAG, "changeState: module '%s' not found", module.c_str());
        return false;
    }
    const std::string& old_state = it->second.current_state;

    if (old_state == new_state) {
        ESP_LOGW(TAG, "Module '%s' attempted invalid transition: %s -> %s (no change)",
                 module.c_str(), old_state.c_str(), new_state.c_str());
        return false;
    }

    if (!force_skip_prereqs && !prereqsSatisfied_locked(module, new_state)) {
        ESP_LOGW(TAG, "Module '%s': transition '%s' -> '%s' blocked by prerequisites",
                 module.c_str(), old_state.c_str(), new_state.c_str());
        return false;
    }

    it->second.current_state = new_state;
    ESP_LOGI(TAG, "Module '%s' transition: %s -> %s",
             module.c_str(), old_state.c_str(), new_state.c_str());
    return true;
}

// ---------------------------------------------------------------------------
// isAtLeast
// ---------------------------------------------------------------------------
bool StateMachine::isAtLeast(const std::string& module,
                              const std::string& min_state) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = modules_.find(module);
    if (it == modules_.end()) return false;
    const int cur  = it->second.current_rank();
    const int need = it->second.rank(min_state);
    if (need < 0) {
        ESP_LOGE(TAG, "isAtLeast: unknown state '%s' for module '%s'",
                 min_state.c_str(), module.c_str());
        return false;
    }
    return cur >= need;
}

// ---------------------------------------------------------------------------
// isInState
// ---------------------------------------------------------------------------
bool StateMachine::isInState(const std::string& module,
                              const std::string& state) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = modules_.find(module);
    return (it != modules_.end()) && (it->second.current_state == state);
}

// ---------------------------------------------------------------------------
// getState
// ---------------------------------------------------------------------------
const std::string& StateMachine::getState(const std::string& module) {
    static const std::string empty;
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = modules_.find(module);
    return (it != modules_.end()) ? it->second.current_state : empty;
}

// ---------------------------------------------------------------------------
// advance (all modules)
// ---------------------------------------------------------------------------
void StateMachine::advance() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::unordered_map<std::string, std::string> current_map;
    for (const auto& kv : modules_) current_map[kv.first] = kv.second.current_state;

    bool changed;
    do {
        changed = false;
        for (auto& kv : modules_) {
            const std::string& module = kv.first;
            ModuleState& mod_state = kv.second;
            const std::string& current = mod_state.current_state;

            std::vector<std::string> ordered;
            if (module == "shelfbot")          ordered = orderedStates(ShelfbotState());
            else if (module == "led_control")  ordered = orderedStates(LedControlState());
            else if (module == "motor_control")ordered = orderedStates(MotorControlState());
            else if (module == "sensor_control")ordered = orderedStates(SensorControlState());
            else if (module == "wifi_manager") ordered = orderedStates(WifiManagerState());
            else if (module == "network_service") ordered = orderedStates(NetworkServiceState());
            else if (module == "microros_sync")ordered = orderedStates(MicrorosState());
            else if (module == "agent")        ordered = orderedStates(AgentState());
            else if (module == "time_sync")    ordered = orderedStates(TimeSyncState());
            else continue;

            int cur_idx = -1;
            for (size_t i = 0; i < ordered.size(); ++i)
                if (ordered[i] == current) { cur_idx = i; break; }
            if (cur_idx < 0) continue;

            for (size_t next_idx = cur_idx + 1; next_idx < ordered.size(); ++next_idx) {
                const std::string& next_state = ordered[next_idx];
                if (!prereqsSatisfied_locked(module, next_state)) continue;
                if (!::is_allowed_transition(module, next_state, current_map)) continue;

                mod_state.current_state = next_state;
                ESP_LOGI(TAG, "advance: %s -> %s", module.c_str(), next_state.c_str());
                current_map[module] = next_state;
                changed = true;
                break;
            }
        }
    } while (changed);
}

// ---------------------------------------------------------------------------
// advance (single module)
// ---------------------------------------------------------------------------
void StateMachine::advance(const std::string& module) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::unordered_map<std::string, std::string> current_map;
    for (const auto& kv : modules_) current_map[kv.first] = kv.second.current_state;

    auto it = modules_.find(module);
    if (it == modules_.end()) {
        ESP_LOGW(TAG, "advance: module '%s' not found", module.c_str());
        return;
    }

    ModuleState& mod_state = it->second;
    const std::string& current = mod_state.current_state;

    std::vector<std::string> ordered;
    if (module == "shelfbot")          ordered = orderedStates(ShelfbotState());
    else if (module == "led_control")  ordered = orderedStates(LedControlState());
    else if (module == "motor_control")ordered = orderedStates(MotorControlState());
    else if (module == "sensor_control")ordered = orderedStates(SensorControlState());
    else if (module == "wifi_manager") ordered = orderedStates(WifiManagerState());
    else if (module == "network_service") ordered = orderedStates(NetworkServiceState());
    else if (module == "microros_sync")ordered = orderedStates(MicrorosState());
    else if (module == "agent")        ordered = orderedStates(AgentState());
    else if (module == "time_sync")    ordered = orderedStates(TimeSyncState());
    else return;

    int cur_idx = -1;
    for (size_t i = 0; i < ordered.size(); ++i)
        if (ordered[i] == current) { cur_idx = i; break; }
    if (cur_idx < 0) return;

    for (size_t next_idx = cur_idx + 1; next_idx < ordered.size(); ++next_idx) {
        const std::string& next_state = ordered[next_idx];
        if (!prereqsSatisfied_locked(module, next_state)) continue;
        if (!::is_allowed_transition(module, next_state, current_map)) continue;

        mod_state.current_state = next_state;
        ESP_LOGI(TAG, "advance: %s -> %s", module.c_str(), next_state.c_str());
        return;
    }
}

// ---------------------------------------------------------------------------
// recover – reset error/recovery states
// ---------------------------------------------------------------------------
void StateMachine::recover() {
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto& kv : modules_) {
        const std::string& module = kv.first;
        const std::string& state = kv.second.current_state;

        if (module == "microros_sync" && (state == "error" || state == "recovering")) {
            changeState(module, "disconnected", true);
        } else if (module == "agent" && state == "error") {
            changeState(module, "offline", true);
        } else if (module == "network_service" && state == "error") {
            changeState(module, "off", true);
        } else if (module == "wifi_manager" && state == "error") {
            changeState(module, "disconnected", true);
        } else if (module == "shelfbot" && state == "error") {
            changeState(module, "setup", true);
        }
    }

    // WiFi disconnect forces network_service and time_sync to safe states
    auto it_wifi = modules_.find("wifi_manager");
    if (it_wifi != modules_.end() && it_wifi->second.current_state == "disconnected") {
        auto it_net = modules_.find("network_service");
        if (it_net != modules_.end() && it_net->second.current_state != "off") {
            changeState("network_service", "off", true);
            ESP_LOGW(TAG, "recover: network_service -> off (wifi disconnected)");
        }
        auto it_time = modules_.find("time_sync");
        if (it_time != modules_.end() && it_time->second.current_state != "unsynced") {
            changeState("time_sync", "unsynced", true);
            ESP_LOGW(TAG, "recover: time_sync -> unsynced (wifi disconnected)");
        }
    }
}

// ---------------------------------------------------------------------------
// Status dump task
// ---------------------------------------------------------------------------
void StateMachine::dumpAllStates() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (modules_.empty()) { ESP_LOGI(TAG, "No modules registered"); return; }
    ESP_LOGI(TAG, "========== State Machine Status ==========");
    for (const auto& kv : modules_)
        ESP_LOGI(TAG, "  %-24s : %s", kv.first.c_str(), kv.second.current_state.c_str());
    ESP_LOGI(TAG, "==========================================");
}

void StateMachine::status_dump_task(void* /*arg*/) {
    while (task_running_) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        dumpAllStates();
    }
    status_task_handle_ = nullptr;
    vTaskDelete(nullptr);
}