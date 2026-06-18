// state_machine.cpp
//
// Locking model
// -------------
// All public methods attempt to take the internal mutex with a 100 ms timeout.
// If the mutex cannot be acquired within that time, the method logs an error
// and returns a safe default (false, empty string, or does nothing).
// This prevents indefinite blocking and makes the system robust against
// accidental long holds by other tasks.
//
// Internal helpers (_locked suffix) require the caller to hold the mutex.
// They are not protected by the timeout mechanism.

#include "state_machine.hpp"
#include "state_machine_lifecycle.hpp"
#include <esp_timer.h>
#include <esp_log.h>

static const char* TAG = "StateMachine";

// ---------------------------------------------------------------------------
// Static member definitions
// ---------------------------------------------------------------------------
SemaphoreHandle_t StateMachine::mutex_ = nullptr;
std::unordered_map<std::string, StateMachine::ModuleState> StateMachine::modules_;
std::unordered_map<
    std::string,
    std::unordered_map<std::string, std::vector<StateMachine::Prerequisite>>
> StateMachine::prerequisites_;
TaskHandle_t StateMachine::status_task_handle_ = nullptr;
bool         StateMachine::task_running_       = false;

// ---------------------------------------------------------------------------
// Mutex helpers with timeout
// ---------------------------------------------------------------------------
bool StateMachine::takeMutex() {
    if (mutex_ == nullptr) {
        ESP_LOGE(TAG, "takeMutex: mutex not initialized");
        return false;
    }
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "takeMutex: timeout (100 ms) - lock held by another task");
        return false;
    }
    return true;
}

void StateMachine::giveMutex() {
    if (mutex_) xSemaphoreGive(mutex_);
}

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------
void StateMachine::init() {
    if (task_running_) {
        ESP_LOGW(TAG, "init() called again — already running, ignoring");
        return;
    }

    // Create the mutex
    if (mutex_ == nullptr) {
        mutex_ = xSemaphoreCreateMutex();
        configASSERT(mutex_ != nullptr);
        ESP_LOGI(TAG, "StateMachine mutex created");
    }

    task_running_ = true;
    const BaseType_t ret = xTaskCreate(
        status_dump_task, "sm_dump", 4096, nullptr,
        tskIDLE_PRIORITY + 1, &status_task_handle_);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create status dump task (ret=%d) — state dumps disabled",
                 (int)ret);
        task_running_ = false;
        status_task_handle_ = nullptr;
    } else {
        ESP_LOGI(TAG, "StateMachine initialised — status dump every 10 s");
    }
}

// ---------------------------------------------------------------------------
// setInitial
// ---------------------------------------------------------------------------
bool StateMachine::setInitial(const std::string& module,
                              const std::string& initial_state,
                              const std::vector<std::string>& ordered_states) {
    if (!takeMutex()) return false;

    bool result = false;
    if (modules_.find(module) != modules_.end()) {
        ESP_LOGW(TAG, "setInitial: module '%s' already registered — ignoring duplicate",
                 module.c_str());
    } else {
        if (!ordered_states.empty()) {
            bool found = false;
            for (const auto& s : ordered_states)
                if (s == initial_state) { found = true; break; }
            if (!found)
                ESP_LOGE(TAG, "setInitial: module '%s' initial_state '%s' not in ordered_states",
                         module.c_str(), initial_state.c_str());
        }
        modules_.emplace(module, ModuleState(initial_state, ordered_states));
        ESP_LOGI(TAG, "Registered module '%-20s' initial state: %s",
                 module.c_str(), initial_state.c_str());
        result = true;
    }
    giveMutex();
    return result;
}

// ---------------------------------------------------------------------------
// registerPrerequisite
// ---------------------------------------------------------------------------
void StateMachine::registerPrerequisite(const std::string& module,
                                         const std::string& target_state,
                                         const Prerequisite& prereq) {
    if (!takeMutex()) return;

    if (modules_.find(module) == modules_.end())
        ESP_LOGE(TAG, "registerPrerequisite: subject module '%s' not yet registered",
                 module.c_str());
    if (modules_.find(prereq.prereq_module) == modules_.end())
        ESP_LOGE(TAG, "registerPrerequisite: prereq module '%s' not yet registered",
                 prereq.prereq_module.c_str());
    prerequisites_[module][target_state].push_back(prereq);
    ESP_LOGI(TAG, "Prereq: '%s'->'%s' requires '%s'>='%s'",
             module.c_str(), target_state.c_str(),
             prereq.prereq_module.c_str(), prereq.min_state.c_str());

    giveMutex();
}

// ---------------------------------------------------------------------------
// prereqsSatisfied_locked  (mutex must be held)
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
            ESP_LOGW(TAG, "prereq check: module '%s' not registered "
                     "(blocking '%s'->'%s')",
                     prereq.prereq_module.c_str(), module.c_str(), new_state.c_str());
            return false;
        }
        const ModuleState& ps       = pmod_it->second;
        const int          cur_rank = ps.rank(ps.current_state);
        const int          need_rank= ps.rank(prereq.min_state);
        if (need_rank < 0) {
            ESP_LOGE(TAG, "prereq: min_state '%s' has no rank in module '%s'",
                     prereq.min_state.c_str(), prereq.prereq_module.c_str());
            return false;
        }
        if (cur_rank < need_rank) {
            ESP_LOGD(TAG, "prereq BLOCK: '%s'->'%s' needs '%s'>='%s'(rank %d) "
                     "current='%s'(rank %d)",
                     module.c_str(), new_state.c_str(),
                     prereq.prereq_module.c_str(), prereq.min_state.c_str(), need_rank,
                     ps.current_state.c_str(), cur_rank);
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// changeState_locked  (private — mutex must already be held by caller)
// ---------------------------------------------------------------------------
bool StateMachine::changeState_locked(const std::string& module,
                                      const std::string& new_state,
                                      bool force_skip_prereqs) {
    auto it = modules_.find(module);
    if (it == modules_.end()) {
        ESP_LOGE(TAG, "changeState: module '%s' not found", module.c_str());
        return false;
    }

    const std::string& old_state = it->second.current_state;

    // No-op guard: never log or record a transition to the same state.
    if (old_state == new_state) {
        ESP_LOGD(TAG, "changeState: '%s' already in '%s' — no-op",
                 module.c_str(), new_state.c_str());
        return false;
    }

    if (!force_skip_prereqs && !prereqsSatisfied_locked(module, new_state)) {
        ESP_LOGW(TAG, "changeState: '%s' %s->%s BLOCKED by prerequisites",
                 module.c_str(), old_state.c_str(), new_state.c_str());
        return false;
    }

    it->second.current_state = new_state;
    ESP_LOGI(TAG, "Transition: %-20s  %s -> %s",
             module.c_str(), old_state.c_str(), new_state.c_str());
    return true;
}

// ---------------------------------------------------------------------------
// changeState  (public — self-locking with timeout)
// ---------------------------------------------------------------------------
bool StateMachine::changeState(const std::string& module,
                               const std::string& new_state,
                               bool force_skip_prereqs) {
    if (!takeMutex()) return false;
    bool result = changeState_locked(module, new_state, force_skip_prereqs);
    giveMutex();
    return result;
}

// ---------------------------------------------------------------------------
// canTransition
// ---------------------------------------------------------------------------
bool StateMachine::canTransition(const std::string& module,
                                  const std::string& new_state) {
    if (!takeMutex()) return false;
    bool result = prereqsSatisfied_locked(module, new_state);
    giveMutex();
    return result;
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
        if (!takeMutex()) {
            // If we can't get the lock, wait and try again
            vTaskDelay(poll);
            continue;
        }
        bool satisfied = prereqsSatisfied_locked(module, target_state);
        giveMutex();
        if (satisfied) return true;
        if ((xTaskGetTickCount() - start) >= timeout) {
            ESP_LOGW(TAG, "waitForPrerequisites: timeout (%u ms) for '%s'->'%s'",
                     (unsigned)timeout_ms, module.c_str(), target_state.c_str());
            return false;
        }
        vTaskDelay(poll);
    }
}

// ---------------------------------------------------------------------------
// isAtLeast
// ---------------------------------------------------------------------------
bool StateMachine::isAtLeast(const std::string& module,
                              const std::string& min_state) {
    if (!takeMutex()) return false;
    auto it = modules_.find(module);
    if (it == modules_.end()) {
        giveMutex();
        return false;
    }
    const int cur  = it->second.current_rank();
    const int need = it->second.rank(min_state);
    if (need < 0) {
        ESP_LOGE(TAG, "isAtLeast: unknown state '%s' for module '%s'",
                 min_state.c_str(), module.c_str());
        giveMutex();
        return false;
    }
    bool result = cur >= need;
    giveMutex();
    return result;
}

// ---------------------------------------------------------------------------
// isInState
// ---------------------------------------------------------------------------
bool StateMachine::isInState(const std::string& module, const std::string& state) {
    if (!takeMutex()) return false;
    auto it = modules_.find(module);
    bool result = (it != modules_.end()) && (it->second.current_state == state);
    giveMutex();
    return result;
}

// ---------------------------------------------------------------------------
// getState
// ---------------------------------------------------------------------------
const std::string& StateMachine::getState(const std::string& module) {
    static const std::string empty;
    if (!takeMutex()) return empty;
    auto it = modules_.find(module);
    if (it != modules_.end()) {
        const std::string& result = it->second.current_state;
        giveMutex();
        return result;
    }
    giveMutex();
    return empty;
}

// ---------------------------------------------------------------------------
// Internal helper: module name → ordered state list
// ---------------------------------------------------------------------------
static std::vector<std::string> getOrderedStates(const std::string& module) {
    if      (module == "shelfbot")        return orderedStates(ShelfbotState());
    else if (module == "led_control")     return orderedStates(LedControlState());
    else if (module == "motor_control")   return orderedStates(MotorControlState());
    else if (module == "wifi_manager")    return orderedStates(WifiManagerState());
    else if (module == "network_service") return orderedStates(NetworkServiceState());
    else if (module == "microros_sync")   return orderedStates(MicrorosState());
    else if (module == "agent")           return orderedStates(AgentState());
    else if (module == "time_sync")       return orderedStates(TimeSyncState());
    else if (module == "lidar_sensor")    return orderedStates(LidarSensorState());
    return {};
}

// ---------------------------------------------------------------------------
// advance (all modules) - unused, kept as void
// ---------------------------------------------------------------------------
void StateMachine::advance() {
    if (!takeMutex()) return;

    std::unordered_map<std::string, std::string> current_map;
    current_map.reserve(modules_.size());
    for (const auto& kv : modules_) current_map[kv.first] = kv.second.current_state;

    bool changed;
    do {
        changed = false;
        for (auto& kv : modules_) {
            const std::string& module    = kv.first;
            const std::string& current   = kv.second.current_state;
            const auto         ordered   = getOrderedStates(module);
            if (ordered.empty()) continue;

            int cur_idx = -1;
            for (size_t i = 0; i < ordered.size(); ++i)
                if (ordered[i] == current) { cur_idx = (int)i; break; }
            if (cur_idx < 0) continue;

            for (size_t ni = (size_t)cur_idx + 1; ni < ordered.size(); ++ni) {
                const std::string& next = ordered[ni];
                if (!prereqsSatisfied_locked(module, next)) continue;
                if (!::is_allowed_transition(module, next, current_map)) continue;
                if (changeState_locked(module, next, true)) {
                    current_map[module] = next;
                    changed = true;
                }
                break;
            }
        }
    } while (changed);

    giveMutex();
}

// ---------------------------------------------------------------------------
// advance (single module) - returns bool and logs reason on failure
// ---------------------------------------------------------------------------
bool StateMachine::advance(const std::string& module) {
    if (!takeMutex()) return false;

    auto it = modules_.find(module);
    if (it == modules_.end()) {
        ESP_LOGW(TAG, "advance('%s') FAILED: module not registered", module.c_str());
        giveMutex();
        return false;
    }

    // Build current state map for transition rule checks
    std::unordered_map<std::string, std::string> current_map;
    current_map.reserve(modules_.size());
    for (const auto& kv : modules_) current_map[kv.first] = kv.second.current_state;

    const std::string& current = it->second.current_state;
    const auto ordered = getOrderedStates(module);
    if (ordered.empty()) {
        ESP_LOGW(TAG, "advance('%s') FAILED: no ordered state list", module.c_str());
        giveMutex();
        return false;
    }

    int cur_idx = -1;
    for (size_t i = 0; i < ordered.size(); ++i)
        if (ordered[i] == current) { cur_idx = (int)i; break; }
    if (cur_idx < 0) {
        ESP_LOGE(TAG, "advance('%s') FAILED: current state '%s' not in ordered list",
                 module.c_str(), current.c_str());
        giveMutex();
        return false;
    }

    // Check if already at the last state
    if ((size_t)cur_idx == ordered.size() - 1) {
        ESP_LOGD(TAG, "advance('%s'): already at final state '%s'", module.c_str(), current.c_str());
        giveMutex();
        return false;
    }

    // Try each next state in order
    for (size_t ni = (size_t)cur_idx + 1; ni < ordered.size(); ++ni) {
        const std::string& next = ordered[ni];

        // Check prerequisites
        if (!prereqsSatisfied_locked(module, next)) {
            ESP_LOGW(TAG, "advance('%s') -> '%s' BLOCKED by prerequisites", module.c_str(), next.c_str());
            // Log the first unsatisfied prerequisite for debugging
            continue;
        }

        // Check allowed transitions
        if (!::is_allowed_transition(module, next, current_map)) {
            ESP_LOGW(TAG, "advance('%s') -> '%s' NOT ALLOWED by transition rules", module.c_str(), next.c_str());
            continue;
        }

        // Attempt the transition
        if (changeState_locked(module, next, true)) {
            ESP_LOGI(TAG, "advance('%s') -> '%s' SUCCEEDED", module.c_str(), next.c_str());
            giveMutex();
            return true;
        }
        // If changeState_locked returns false, it already logged the reason
        break; // stop at first eligible transition that fails
    }

    giveMutex();
    return false;
}

// ---------------------------------------------------------------------------
// recover
// ---------------------------------------------------------------------------
void StateMachine::recover() {
    if (!takeMutex()) return;

    for (auto& kv : modules_) {
        const std::string& module = kv.first;
        const std::string& state  = kv.second.current_state;

        if (module == "microros_sync") {
            if (state == "error"             ||
                state == "recovering"        ||
                state == "creating_entities" ||
                state == "connected") {
                if (changeState_locked(module, "disconnected", true))
                    changeState_locked("agent", "offline", true);
            }
        } else if (module == "agent" && state == "error") {
            changeState_locked(module, "offline", true);
        } else if (module == "network_service" && state == "error") {
            changeState_locked(module, "off", true);
        } else if (module == "wifi_manager" && state == "error") {
            changeState_locked(module, "disconnected", true);
        } else if (module == "shelfbot" && state == "error") {
            changeState_locked(module, "setup", true);
        } else if (module == "lidar_sensor" && state == "error") {
            changeState_locked(module, "setup", true);
        }
    }

    // Cascade: wifi disconnected → reset network-dependent modules.
    auto it_wifi = modules_.find("wifi_manager");
    if (it_wifi != modules_.end() &&
        it_wifi->second.current_state == "disconnected") {
        auto it_net = modules_.find("network_service");
        if (it_net != modules_.end() && it_net->second.current_state != "off") {
            if (changeState_locked("network_service", "off", true))
                ESP_LOGW(TAG, "recover: network_service -> off (wifi disconnected)");
        }
        auto it_time = modules_.find("time_sync");
        if (it_time != modules_.end() && it_time->second.current_state != "unsynced") {
            if (changeState_locked("time_sync", "unsynced", true))
                ESP_LOGW(TAG, "recover: time_sync -> unsynced (wifi disconnected)");
        }
    }

    giveMutex();
}

// ---------------------------------------------------------------------------
// Status dump task
// ---------------------------------------------------------------------------
void StateMachine::dumpAllStates() {
    if (!takeMutex()) return;
    if (modules_.empty()) { ESP_LOGI(TAG, "No modules registered"); giveMutex(); return; }
    ESP_LOGI(TAG, "===== State Machine Status =====");
    for (const auto& kv : modules_)
        ESP_LOGI(TAG, "  %-24s : %s", kv.first.c_str(), kv.second.current_state.c_str());
    ESP_LOGI(TAG, "================================");
    giveMutex();
}

void StateMachine::status_dump_task(void* /*arg*/) {
    while (task_running_) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        dumpAllStates();
    }
    status_task_handle_ = nullptr;
    vTaskDelete(nullptr);
}