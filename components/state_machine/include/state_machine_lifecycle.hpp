#ifndef STATE_MACHINE_LIFECYCLE_HPP
#define STATE_MACHINE_LIFECYCLE_HPP

#pragma once
#include <idf_c_includes.hpp>
#include <unordered_map>
#include <string>
#include <vector>
#include <functional>

// ============================================================================
// Module state enumerations
// ============================================================================
enum class ShelfbotState : uint8_t {
    SETUP = 0, INIT, RUNNING, STOPPED, ERROR, SHUTDOWN, COUNT
};
enum class MotorControlState : uint8_t {
    SETUP = 0, INIT, RUNNING, STOPPED, ERROR, DISABLED, COUNT
};
enum class MicrorosState : uint8_t {
    OFF = 0, DISCONNECTED, DISCOVERING, CREATING_ENTITIES, CONNECTED, ERROR, RECOVERING, COUNT
};
enum class WifiManagerState : uint8_t {
    OFF = 0, DISCONNECTED, CONNECTING, CONNECTED, ERROR, COUNT
};
enum class NetworkServiceState : uint8_t {
    OFF = 0, STARTING, MDNS_READY, HTTP_RUNNING, ERROR, COUNT
};
enum class AgentState : uint8_t {
    OFFLINE = 0, DISCOVERED, PING_OK, SESSION_SYNCED, ENTITIES_CREATED, CONNECTED, ERROR, COUNT
};
enum class TimeSyncState : uint8_t {
    UNSYNCED = 0, SYNCED, COUNT
};
enum class LedControlState : uint8_t {
    SETUP = 0, INIT, RUNNING, STOPPED, ERROR, COUNT
};
enum class LidarSensorState : uint8_t {
    SETUP = 0, INIT, RUNNING, STOPPED, ERROR, COUNT
};

// ============================================================================
// stateToString – must be defined for all enums
// ============================================================================
inline const char* stateToString(ShelfbotState s) {
    switch (s) {
        case ShelfbotState::SETUP:    return "setup";
        case ShelfbotState::INIT:     return "init";
        case ShelfbotState::RUNNING:  return "running";
        case ShelfbotState::STOPPED:  return "stopped";
        case ShelfbotState::ERROR:    return "error";
        case ShelfbotState::SHUTDOWN: return "shutdown";
        default:                      return "unknown";
    }
}
inline const char* stateToString(MotorControlState s) {
    switch (s) {
        case MotorControlState::SETUP:    return "setup";
        case MotorControlState::INIT:     return "init";
        case MotorControlState::RUNNING:  return "running";
        case MotorControlState::STOPPED:  return "stopped";
        case MotorControlState::ERROR:    return "error";
        case MotorControlState::DISABLED: return "disabled";
        default:                          return "unknown";
    }
}
inline const char* stateToString(MicrorosState s) {
    switch (s) {
        case MicrorosState::OFF:               return "off";
        case MicrorosState::DISCONNECTED:      return "disconnected";
        case MicrorosState::DISCOVERING:       return "discovering";
        case MicrorosState::CREATING_ENTITIES: return "creating_entities";
        case MicrorosState::CONNECTED:         return "connected";
        case MicrorosState::ERROR:             return "error";
        case MicrorosState::RECOVERING:        return "recovering";
        default:                               return "unknown";
    }
}
inline const char* stateToString(WifiManagerState s) {
    switch (s) {
        case WifiManagerState::OFF:          return "off";
        case WifiManagerState::DISCONNECTED: return "disconnected";
        case WifiManagerState::CONNECTING:   return "connecting";
        case WifiManagerState::CONNECTED:    return "connected";
        case WifiManagerState::ERROR:        return "error";
        default:                             return "unknown";
    }
}
inline const char* stateToString(NetworkServiceState s) {
    switch (s) {
        case NetworkServiceState::OFF:          return "off";
        case NetworkServiceState::STARTING:     return "starting";
        case NetworkServiceState::MDNS_READY:   return "mdns_ready";
        case NetworkServiceState::HTTP_RUNNING: return "http_running";
        case NetworkServiceState::ERROR:        return "error";
        default:                                return "unknown";
    }
}
inline const char* stateToString(AgentState s) {
    switch (s) {
        case AgentState::OFFLINE:          return "offline";
        case AgentState::DISCOVERED:       return "discovered";
        case AgentState::PING_OK:          return "ping_ok";
        case AgentState::SESSION_SYNCED:   return "session_synced";
        case AgentState::ENTITIES_CREATED: return "entities_created";
        case AgentState::CONNECTED:        return "connected";
        case AgentState::ERROR:            return "error";
        default:                           return "unknown";
    }
}
inline const char* stateToString(TimeSyncState s) {
    switch (s) {
        case TimeSyncState::UNSYNCED: return "unsynced";
        case TimeSyncState::SYNCED:   return "synced";
        default:                      return "unknown";
    }
}
inline const char* stateToString(LedControlState s) {
    switch (s) {
        case LedControlState::SETUP:    return "setup";
        case LedControlState::INIT:     return "init";
        case LedControlState::RUNNING:  return "running";
        case LedControlState::STOPPED:  return "stopped";
        case LedControlState::ERROR:    return "error";
        default:                        return "unknown";
    }
}
inline const char* stateToString(LidarSensorState s) {
    switch (s) {
        case LidarSensorState::SETUP:   return "setup";
        case LidarSensorState::INIT:    return "init";
        case LidarSensorState::RUNNING: return "running";
        case LidarSensorState::STOPPED: return "stopped";
        case LidarSensorState::ERROR:   return "error";
        default:                        return "unknown";
    }
}

// ============================================================================
// orderedStates – must be defined for all enums
// ============================================================================
inline std::vector<std::string> orderedStates(ShelfbotState) {
    return {"setup", "init", "running", "stopped", "error", "shutdown"};
}
inline std::vector<std::string> orderedStates(MotorControlState) {
    return {"setup", "init", "running", "stopped", "error", "disabled"};
}
inline std::vector<std::string> orderedStates(MicrorosState) {
    return {"off", "disconnected", "discovering", "creating_entities", "connected", "error", "recovering"};
}
inline std::vector<std::string> orderedStates(WifiManagerState) {
    return {"off", "disconnected", "connecting", "connected", "error"};
}
inline std::vector<std::string> orderedStates(NetworkServiceState) {
    return {"off", "starting", "mdns_ready", "http_running", "error"};
}
inline std::vector<std::string> orderedStates(AgentState) {
    return {"offline", "discovered", "ping_ok", "session_synced", "entities_created", "connected", "error"};
}
inline std::vector<std::string> orderedStates(TimeSyncState) {
    return {"unsynced", "synced"};
}
inline std::vector<std::string> orderedStates(LedControlState) {
    return {"setup", "init", "running", "stopped", "error"};
}
inline std::vector<std::string> orderedStates(LidarSensorState) {
    return {"setup", "init", "running", "stopped", "error"};
}

// ============================================================================
// Helper: get rank of a state string for a given module
// ============================================================================
static int get_state_rank(const std::string& module, const std::string& state) {
    if (module == "shelfbot") {
        auto v = orderedStates(ShelfbotState());
        for (size_t i = 0; i < v.size(); ++i) if (v[i] == state) return (int)i;
    } else if (module == "led_control") {
        auto v = orderedStates(LedControlState());
        for (size_t i = 0; i < v.size(); ++i) if (v[i] == state) return (int)i;
    } else if (module == "motor_control") {
        auto v = orderedStates(MotorControlState());
        for (size_t i = 0; i < v.size(); ++i) if (v[i] == state) return (int)i;
    } else if (module == "wifi_manager") {
        auto v = orderedStates(WifiManagerState());
        for (size_t i = 0; i < v.size(); ++i) if (v[i] == state) return (int)i;
    } else if (module == "network_service") {
        auto v = orderedStates(NetworkServiceState());
        for (size_t i = 0; i < v.size(); ++i) if (v[i] == state) return (int)i;
    } else if (module == "microros_sync") {
        auto v = orderedStates(MicrorosState());
        for (size_t i = 0; i < v.size(); ++i) if (v[i] == state) return (int)i;
    } else if (module == "agent") {
        auto v = orderedStates(AgentState());
        for (size_t i = 0; i < v.size(); ++i) if (v[i] == state) return (int)i;
    } else if (module == "time_sync") {
        auto v = orderedStates(TimeSyncState());
        for (size_t i = 0; i < v.size(); ++i) if (v[i] == state) return (int)i;
    } else if (module == "lidar_sensor") {
        auto v = orderedStates(LidarSensorState());
        for (size_t i = 0; i < v.size(); ++i) if (v[i] == state) return (int)i;
    }
    return -1;
}

// ============================================================================
// Transition rules with rank‑based prerequisites
// ============================================================================
struct TransitionRule {
    std::string module;
    std::string from_state;
    std::string to_state;
    std::vector<std::pair<std::string, int>> prerequisites;
};

inline const std::vector<TransitionRule> ALLOWED_TRANSITIONS = {
    // Shelfbot
    {"shelfbot", "setup", "init", {}},
    {"shelfbot", "init", "running", {{"wifi_manager", 3}, {"network_service", 2}, {"time_sync", 1}}},
    {"shelfbot", "running", "error", {}},
    {"shelfbot", "error", "shutdown", {}},

    // WiFi
    {"wifi_manager", "off", "disconnected", {}},
    {"wifi_manager", "disconnected", "connecting", {}},
    {"wifi_manager", "connecting", "connected", {}},
    {"wifi_manager", "connected", "disconnected", {}},
    {"wifi_manager", "connected", "error", {}},
    {"wifi_manager", "error", "disconnected", {}},

    // Network service
    {"network_service", "off", "starting", {{"wifi_manager", 3}}},
    {"network_service", "starting", "mdns_ready", {}},
    {"network_service", "mdns_ready", "http_running", {}},
    {"network_service", "http_running", "off", {{"wifi_manager", 1}}},
    {"network_service", "mdns_ready", "off", {{"wifi_manager", 1}}},
    {"network_service", "starting", "off", {{"wifi_manager", 1}}},

    // Time sync
    {"time_sync", "unsynced", "synced", {{"wifi_manager", 3}, {"network_service", 2}}},
    {"time_sync", "synced", "unsynced", {}},

    // micro-ROS
    {"microros_sync", "disconnected", "discovering", {{"wifi_manager", 3}, {"network_service", 2}}},
    {"microros_sync", "discovering", "creating_entities", {{"agent", 3}, {"time_sync", 1}}},
    {"microros_sync", "creating_entities", "connected", {}},
    {"microros_sync", "connected", "disconnected", {}},
    {"microros_sync", "connected", "recovering", {}},
    {"microros_sync", "recovering", "disconnected", {}},
    {"microros_sync", "discovering", "disconnected", {{"wifi_manager", 1}}},
    {"microros_sync", "creating_entities", "disconnected", {{"time_sync", 0}}},
    {"microros_sync", "connected", "disconnected", {{"wifi_manager", 1}}},

    // Agent
    {"agent", "offline", "discovered", {{"microros_sync", 2}}},
    {"agent", "discovered", "ping_ok", {}},
    {"agent", "ping_ok", "session_synced", {}},
    {"agent", "session_synced", "entities_created", {{"microros_sync", 3}}},
    {"agent", "entities_created", "connected", {{"microros_sync", 4}}},
    {"agent", "discovered", "offline", {{"microros_sync", 1}}},
    {"agent", "ping_ok", "offline", {{"microros_sync", 1}}},
    {"agent", "session_synced", "offline", {{"microros_sync", 1}}},
    {"agent", "entities_created", "offline", {{"microros_sync", 1}}},
    {"agent", "connected", "offline", {{"microros_sync", 1}}},

    // Motor control
    {"motor_control", "setup", "init", {{"shelfbot", 2}}},
    {"motor_control", "init", "running", {}},
    {"motor_control", "running", "stopped", {}},
    {"motor_control", "stopped", "setup", {}},

    // LED control
    {"led_control", "setup", "init", {{"shelfbot", 2}}},
    {"led_control", "init", "running", {}},
    {"led_control", "running", "stopped", {}},
    {"led_control", "stopped", "setup", {}},

    // LiDAR sensor
    {"lidar_sensor", "setup", "init", {{"shelfbot", 2}}},
    {"lidar_sensor", "init", "running", {}},
    {"lidar_sensor", "running", "stopped", {}},
    {"lidar_sensor", "stopped", "setup", {}},
    {"lidar_sensor", "running", "error", {}},
    {"lidar_sensor", "error", "setup", {}},
};

// ============================================================================
// Check if a transition is allowed given current states
// ============================================================================
inline bool is_allowed_transition(const std::string& module,
                                  const std::string& to_state,
                                  const std::unordered_map<std::string, std::string>& current_states) {
    std::string from_state = current_states.count(module) ? current_states.at(module) : "";
    for (const auto& rule : ALLOWED_TRANSITIONS) {
        if (rule.module == module && rule.from_state == from_state && rule.to_state == to_state) {
            bool ok = true;
            for (const auto& prereq : rule.prerequisites) {
                auto it = current_states.find(prereq.first);
                if (it == current_states.end()) { ok = false; break; }
                int cur_rank = get_state_rank(prereq.first, it->second);
                if (cur_rank < prereq.second) { ok = false; break; }
            }
            if (ok) return true;
        }
    }
    return false;
}

// ============================================================================
// Get the next allowed state for a module (same as desired if allowed, else current)
// ============================================================================
inline std::string get_next_allowed_state(const std::string& module,
                                          const std::string& desired_state,
                                          const std::unordered_map<std::string, std::string>& current_states) {
    if (is_allowed_transition(module, desired_state, current_states)) {
        return desired_state;
    }
    auto it = current_states.find(module);
    return (it != current_states.end()) ? it->second : "";
}

#endif // STATE_MACHINE_LIFECYCLE_HPP