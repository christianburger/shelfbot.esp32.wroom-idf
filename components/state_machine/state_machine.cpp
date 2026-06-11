#include <state_machine.hpp>
#include <esp_timer.h>

static const char* TAG = "StateMachine";

std::mutex StateMachine::mutex_;
std::unordered_map<std::string, StateMachine::ModuleState> StateMachine::modules_;
std::unordered_map<std::string,
    std::unordered_map<std::string, std::vector<StateMachine::Prerequisite>>> StateMachine::prerequisites_;
TaskHandle_t StateMachine::status_task_handle_ = nullptr;
bool StateMachine::task_running_ = false;

void StateMachine::init() {
    if (task_running_) {
        ESP_LOGW(TAG, "StateMachine already initialised");
        return;
    }
    task_running_ = true;
    BaseType_t ret = xTaskCreate(status_dump_task, "state_dump", 4096, nullptr,
                                 tskIDLE_PRIORITY + 1, &status_task_handle_);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create status dump task");
        task_running_ = false;
    } else {
        ESP_LOGI(TAG, "StateMachine initialised, status logged every 10 seconds");
    }
}

bool StateMachine::setInitial(const std::string& module,
                              const std::string& initial_state,
                              const std::vector<std::string>& ordered_states) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (modules_.find(module) != modules_.end()) {
        ESP_LOGW(TAG, "Module '%s' already exists", module.c_str());
        return false;
    }
    modules_.emplace(module, ModuleState(initial_state, ordered_states));
    ESP_LOGI(TAG, "Module '%s' initial: %s (ranks: %zu)",
             module.c_str(), initial_state.c_str(), ordered_states.size());
    return true;
}

void StateMachine::registerPrerequisite(const std::string& module,
                                         const std::string& target_state,
                                         const Prerequisite& prereq) {
    std::lock_guard<std::mutex> lock(mutex_);
    prerequisites_[module][target_state].push_back(prereq);
    ESP_LOGD(TAG, "Prereq: %s can't go to '%s' unless %s >= '%s'",
             module.c_str(), target_state.c_str(),
             prereq.prereq_module.c_str(), prereq.min_state.c_str());
}

bool StateMachine::prereqsSatisfied_locked(const std::string& module,
                                           const std::string& new_state) {
    auto mod_it = prerequisites_.find(module);
    if (mod_it == prerequisites_.end()) return true;
    auto state_it = mod_it->second.find(new_state);
    if (state_it == mod_it->second.end()) return true;

    for (const auto& prereq : state_it->second) {
        auto pmod_it = modules_.find(prereq.prereq_module);
        if (pmod_it == modules_.end()) {
            ESP_LOGE(TAG, "Prerequisite module '%s' unknown", prereq.prereq_module.c_str());
            return false;
        }
        const ModuleState& pstate = pmod_it->second;
        if (pstate.rank(pstate.current_state) < pstate.rank(prereq.min_state)) {
            ESP_LOGD(TAG, "%s needs %s >= %s (currently %s)",
                     module.c_str(), prereq.prereq_module.c_str(),
                     prereq.min_state.c_str(), pstate.current_state.c_str());
            return false;
        }
    }
    return true;
}

bool StateMachine::canTransition(const std::string& module, const std::string& new_state) {
    std::lock_guard<std::mutex> lock(mutex_);
    return prereqsSatisfied_locked(module, new_state);
}

bool StateMachine::waitForPrerequisites(const std::string& module,
                                         const std::string& target_state,
                                         uint32_t timeout_ms,
                                         uint32_t poll_ms) {
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    TickType_t poll_ticks = pdMS_TO_TICKS(poll_ms);

    while (true) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (prereqsSatisfied_locked(module, target_state))
                return true;
        }
        if (xTaskGetTickCount() - start >= timeout_ticks)
            break;
        vTaskDelay(poll_ticks);
    }
    ESP_LOGW(TAG, "Timeout waiting for %s to reach '%s'", module.c_str(), target_state.c_str());
    return false;
}

bool StateMachine::changeState(const std::string& module,
                               const std::string& new_state,
                               bool force_skip_prereqs) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = modules_.find(module);
    if (it == modules_.end()) {
        ESP_LOGE(TAG, "Module '%s' not found", module.c_str());
        return false;
    }
    const std::string& old = it->second.current_state;
    if (old == new_state) return true;

    if (!force_skip_prereqs && !prereqsSatisfied_locked(module, new_state)) {
        ESP_LOGW(TAG, "%s: cannot go to '%s' – prerequisites not met", module.c_str(), new_state.c_str());
        return false;
    }

    it->second.current_state = new_state;
    ESP_LOGI(TAG, "%s: %s -> %s", module.c_str(), old.c_str(), new_state.c_str());
    return true;
}

bool StateMachine::isInState(const std::string& module, const std::string& state) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = modules_.find(module);
    return it != modules_.end() && it->second.current_state == state;
}

bool StateMachine::isAtLeast(const std::string& module, const std::string& min_state) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = modules_.find(module);
    if (it == modules_.end()) return false;
    return it->second.current_rank() >= it->second.rank(min_state);
}

const std::string& StateMachine::getState(const std::string& module) {
    static const std::string empty;
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = modules_.find(module);
    return (it != modules_.end()) ? it->second.current_state : empty;
}

void StateMachine::dumpAllStates() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (modules_.empty()) {
        ESP_LOGI(TAG, "No modules registered yet");
        return;
    }
    ESP_LOGI(TAG, "========== State Machine Status ==========");
    for (const auto& pair : modules_) {
        ESP_LOGI(TAG, "  %-20s : %s", pair.first.c_str(), pair.second.current_state.c_str());
    }
    ESP_LOGI(TAG, "===========================================");
}

void StateMachine::status_dump_task(void* arg) {
    (void)arg;
    ESP_LOGI(TAG, "Status dump task started");
    while (task_running_) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        dumpAllStates();
    }
    ESP_LOGI(TAG, "Status dump task stopping");
    status_task_handle_ = nullptr;
    vTaskDelete(nullptr);
}