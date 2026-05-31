#include <state_machine.hpp>

static auto TAG = "StateMachine";

// Static member definitions
std::mutex StateMachine::mutex_;
std::unordered_map<std::string, StateMachine::ModuleState> StateMachine::modules_;
TaskHandle_t StateMachine::status_task_handle_ = nullptr;
bool StateMachine::task_running_ = false;

void StateMachine::init() {
    if (task_running_) {
        ESP_LOGW(TAG, "StateMachine already initialised");
        return;
    }

    task_running_ = true;
    BaseType_t ret = xTaskCreate(
        status_dump_task,
        "state_dump",
        4096,
        nullptr,
        tskIDLE_PRIORITY + 1,
        &status_task_handle_
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create status dump task");
        task_running_ = false;
    } else {
        ESP_LOGI(TAG, "StateMachine initialised, status logged every 10 seconds");
    }
}

bool StateMachine::setInitial(const std::string& module, const std::string& initial_state) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = modules_.find(module);
    if (it != modules_.end()) {
        ESP_LOGW(TAG, "Module '%s' already exists (state=%s), ignoring setInitial",
                 module.c_str(), it->second.current_state.c_str());
        return false;
    }
    modules_[module] = ModuleState(initial_state);
    ESP_LOGI(TAG, "Module '%s' initial state: %s", module.c_str(), initial_state.c_str());
    return true;
}

bool StateMachine::changeState(const std::string& module, const std::string& new_state) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = modules_.find(module);
    if (it == modules_.end()) {
        ESP_LOGE(TAG, "Module '%s' not found, cannot change state to '%s'",
                 module.c_str(), new_state.c_str());
        return false;
    }
    std::string old_state = it->second.current_state;
    if (old_state == new_state) {
        return true;
    }
    it->second.current_state = new_state;
    ESP_LOGI(TAG, "Module '%s' transition: %s -> %s",
             module.c_str(), old_state.c_str(), new_state.c_str());
    return true;
}

const std::string& StateMachine::getState(const std::string& module) {
    static const std::string empty;
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = modules_.find(module);
    if (it == modules_.end()) {
        return empty;
    }
    return it->second.current_state;
}

void StateMachine::dumpAllStates() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (modules_.empty()) {
        ESP_LOGI(TAG, "No modules registered yet");
        return;
    }

    ESP_LOGI(TAG, "========== State Machine Status ==========");
    for (const auto& pair : modules_) {
        ESP_LOGI(TAG, "  %-20s : %s",
                 pair.first.c_str(),
                 pair.second.current_state.c_str());
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