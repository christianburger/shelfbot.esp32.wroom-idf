#include <state_machine.hpp>
#include <state_machine_lifecycle.hpp>
#include <esp_timer.h>

static const char* TAG = "StateMachine";

std::unordered_map<std::string, StateMachine::ModuleState> StateMachine::modules_;
std::mutex StateMachine::mutex_;

// ----------------------------------------------------------------------------
// String to enum conversion (forward declarations from lifecycle header)
// ----------------------------------------------------------------------------
static ShelfbotState toShelfbotState(const std::string& s) {
    if (s == "starting") return ShelfbotState::STARTING;
    if (s == "running")  return ShelfbotState::RUNNING;
    if (s == "error")    return ShelfbotState::ERROR;
    if (s == "shutdown") return ShelfbotState::SHUTDOWN;
    return ShelfbotState::ERROR;
}
static MotorControlState toMotorControlState(const std::string& s) {
    if (s == "off")      return MotorControlState::OFF;
    if (s == "idle")     return MotorControlState::IDLE;
    if (s == "moving")   return MotorControlState::MOVING;
    if (s == "error")    return MotorControlState::ERROR;
    if (s == "disabled") return MotorControlState::DISABLED;
    return MotorControlState::ERROR;
}
static SensorControlState toSensorControlState(const std::string& s) {
    if (s == "off")      return SensorControlState::OFF;
    if (s == "idle")     return SensorControlState::IDLE;
    if (s == "scanning") return SensorControlState::SCANNING;
    if (s == "error")    return SensorControlState::ERROR;
    if (s == "disabled") return SensorControlState::DISABLED;
    return SensorControlState::ERROR;
}
static MicrorosState toMicrorosState(const std::string& s) {
    if (s == "off")          return MicrorosState::OFF;
    if (s == "discovering")  return MicrorosState::DISCOVERING;
    if (s == "connected")    return MicrorosState::CONNECTED;
    if (s == "error")        return MicrorosState::ERROR;
    if (s == "disconnected") return MicrorosState::DISCONNECTED;
    return MicrorosState::ERROR;
}
static WifiManagerState toWifiManagerState(const std::string& s) {
    if (s == "off")          return WifiManagerState::OFF;
    if (s == "connecting")   return WifiManagerState::CONNECTING;
    if (s == "connected")    return WifiManagerState::CONNECTED;
    if (s == "error")        return WifiManagerState::ERROR;
    if (s == "disconnected") return WifiManagerState::DISCONNECTED;
    return WifiManagerState::ERROR;
}

// ----------------------------------------------------------------------------
// Helper: check if a transition is allowed for a given module and state strings
// ----------------------------------------------------------------------------
static bool is_transition_allowed(const std::string& module_name,
                                  const std::string& from_state_str,
                                  const std::string& to_state_str) {
    if (module_name == "shelfbot") {
        ShelfbotState from = toShelfbotState(from_state_str);
        ShelfbotState to   = toShelfbotState(to_state_str);
        if (from == ShelfbotState::ERROR || to == ShelfbotState::ERROR) return false;
        return shelfbot_transitions[static_cast<size_t>(from)][static_cast<size_t>(to)];
    }
    else if (module_name == "motor_control") {
        MotorControlState from = toMotorControlState(from_state_str);
        MotorControlState to   = toMotorControlState(to_state_str);
        if (from == MotorControlState::ERROR || to == MotorControlState::ERROR) return false;
        return motor_control_transitions[static_cast<size_t>(from)][static_cast<size_t>(to)];
    }
    else if (module_name == "sensor_control") {
        SensorControlState from = toSensorControlState(from_state_str);
        SensorControlState to   = toSensorControlState(to_state_str);
        if (from == SensorControlState::ERROR || to == SensorControlState::ERROR) return false;
        return sensor_control_transitions[static_cast<size_t>(from)][static_cast<size_t>(to)];
    }
    else if (module_name == "microros_sync") {
        MicrorosState from = toMicrorosState(from_state_str);
        MicrorosState to   = toMicrorosState(to_state_str);
        if (from == MicrorosState::ERROR || to == MicrorosState::ERROR) return false;
        return microros_transitions[static_cast<size_t>(from)][static_cast<size_t>(to)];
    }
    else if (module_name == "wifi_manager") {
        WifiManagerState from = toWifiManagerState(from_state_str);
        WifiManagerState to   = toWifiManagerState(to_state_str);
        if (from == WifiManagerState::ERROR || to == WifiManagerState::ERROR) return false;
        return wifi_manager_transitions[static_cast<size_t>(from)][static_cast<size_t>(to)];
    }
    return false; // unknown module
}

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------
bool StateMachine::init() {
    std::lock_guard<std::mutex> lock(mutex_);
    modules_.clear();
    const std::vector<std::string> module_names = {
        "shelfbot", "motor_control", "sensor_control", "microros_sync", "wifi_manager"
    };
    for (const auto& mod : module_names) {
        ModuleState ms;
        ms.current_state_str = "";
        ms.current_timestamp = 0;
        ms.previous_state_str = "";
        ms.previous_timestamp = 0;
        modules_[mod] = ms;
    }
    ESP_LOGI(TAG, "State machine initialized with %zu modules", modules_.size());
    return true;
}

bool StateMachine::setInitial(const std::string& module_name, const std::string& initial_state) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = modules_.find(module_name);
    if (it == modules_.end()) {
        ESP_LOGE(TAG, "Unknown module '%s'", module_name.c_str());
        return false;
    }
    // Validate that the state string is a valid state for this module
    bool valid = false;
    if (module_name == "shelfbot") {
        valid = (toShelfbotState(initial_state) != ShelfbotState::ERROR);
    } else if (module_name == "motor_control") {
        valid = (toMotorControlState(initial_state) != MotorControlState::ERROR);
    } else if (module_name == "sensor_control") {
        valid = (toSensorControlState(initial_state) != SensorControlState::ERROR);
    } else if (module_name == "microros_sync") {
        valid = (toMicrorosState(initial_state) != MicrorosState::ERROR);
    } else if (module_name == "wifi_manager") {
        valid = (toWifiManagerState(initial_state) != WifiManagerState::ERROR);
    }
    if (!valid) {
        ESP_LOGE(TAG, "Invalid initial state '%s' for module '%s'", initial_state.c_str(), module_name.c_str());
        return false;
    }
    it->second.current_state_str = initial_state;
    it->second.current_timestamp = esp_timer_get_time();
    it->second.previous_state_str = "";
    it->second.previous_timestamp = 0;
    ESP_LOGI(TAG, "Module '%s' initial state set to '%s'", module_name.c_str(), initial_state.c_str());
    return true;
}

bool StateMachine::changeState(const std::string& module_name, const std::string& new_state_str) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = modules_.find(module_name);
    if (it == modules_.end()) {
        ESP_LOGE(TAG, "Unknown module '%s'", module_name.c_str());
        return false;
    }
    const std::string& old_state_str = it->second.current_state_str;
    if (old_state_str == new_state_str) {
        ESP_LOGW(TAG, "Module '%s' already in state '%s'", module_name.c_str(), new_state_str.c_str());
        return true;
    }

    // Check transition using the helper
    if (!is_transition_allowed(module_name, old_state_str, new_state_str)) {
        ESP_LOGE(TAG, "Transition from '%s' to '%s' not allowed for module '%s'",
                 old_state_str.c_str(), new_state_str.c_str(), module_name.c_str());
        return false;
    }

    // Perform transition
    ESP_LOGI(TAG, "Module '%s' transition: '%s' -> '%s'",
             module_name.c_str(), old_state_str.c_str(), new_state_str.c_str());
    updateState(module_name, new_state_str);
    return true;
}

std::string StateMachine::getCurrentState(const std::string& module_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = modules_.find(module_name);
    return (it != modules_.end()) ? it->second.current_state_str : "";
}

int64_t StateMachine::getCurrentStateTimestamp(const std::string& module_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = modules_.find(module_name);
    return (it != modules_.end()) ? it->second.current_timestamp : 0;
}

std::string StateMachine::getPreviousState(const std::string& module_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = modules_.find(module_name);
    return (it != modules_.end()) ? it->second.previous_state_str : "";
}

int64_t StateMachine::getPreviousStateTimestamp(const std::string& module_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = modules_.find(module_name);
    return (it != modules_.end()) ? it->second.previous_timestamp : 0;
}

void StateMachine::logCurrentState(const std::string& module_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = modules_.find(module_name);
    if (it != modules_.end()) {
        const auto& mod = it->second;
        ESP_LOGI(TAG, "Module '%s' state='%s' (since %lld µs), prev='%s'",
                 module_name.c_str(),
                 mod.current_state_str.c_str(),
                 (long long)mod.current_timestamp,
                 mod.previous_state_str.c_str());
    } else {
        ESP_LOGW(TAG, "Module '%s' not registered", module_name.c_str());
    }
}

void StateMachine::updateState(const std::string& module_name, const std::string& new_state_str) {
    auto it = modules_.find(module_name);
    if (it == modules_.end()) return;
    ModuleState& mod = it->second;
    mod.previous_state_str = mod.current_state_str;
    mod.previous_timestamp = mod.current_timestamp;
    mod.current_state_str = new_state_str;
    mod.current_timestamp = esp_timer_get_time();
}