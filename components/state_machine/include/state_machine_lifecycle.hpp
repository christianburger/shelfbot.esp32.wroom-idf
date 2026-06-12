#pragma once
#include <idf_c_includes.hpp>

// ---------------------------------------------------------------------------
// Module state enumerations
//
// Every enum is kept in the order the states appear in the module's ordered
// lifecycle list.  stateToString() and orderedStates() are the only places
// that know the string representation — do NOT hard-code state strings
// anywhere else in the firmware.
// ---------------------------------------------------------------------------

enum class ShelfbotState : uint8_t {
    STARTING = 0, RUNNING, ERROR, SHUTDOWN, COUNT
};

enum class MotorControlState : uint8_t {
    OFF = 0, IDLE, MOVING, ERROR, DISABLED, COUNT
};

enum class SensorControlState : uint8_t {
    OFF = 0, IDLE, SCANNING, ERROR, DISABLED, COUNT
};

// micro-ROS session states (the firmware side)
// Ordered from "least ready" to "most ready" for isAtLeast() comparisons.
// OFF and DISCONNECTED share the same readiness band (not connected);
// DISCONNECTED is placed just above OFF so a self-prereq "must be >=
// DISCONNECTED before going to DISCOVERING" correctly gates the first cycle
// (state starts at DISCONNECTED) and every reconnect cycle.
enum class MicrorosState : uint8_t {
    OFF              = 0,
    DISCONNECTED     = 1,
    DISCOVERING      = 2,
    CREATING_ENTITIES= 3,
    CONNECTED        = 4,
    ERROR            = 5,
    COUNT
};

// Wi-Fi manager states
enum class WifiManagerState : uint8_t {
    OFF         = 0,
    DISCONNECTED= 1,
    CONNECTING  = 2,
    CONNECTED   = 3,
    ERROR       = 4,
    COUNT
};

// Network services (mDNS + HTTP)
enum class NetworkServiceState : uint8_t {
    OFF         = 0,
    STARTING    = 1,
    MDNS_READY  = 2,
    HTTP_RUNNING= 3,
    ERROR       = 4,
    COUNT
};

// Remote micro-ROS agent states
// Ordered from "not reachable" to "fully connected".
enum class AgentState : uint8_t {
    OFFLINE         = 0,
    DISCOVERED      = 1,
    PING_OK         = 2,
    SESSION_SYNCED  = 3,
    ENTITIES_CREATED= 4,
    CONNECTED       = 5,
    ERROR           = 6,
    COUNT
};

// Clock synchronisation pseudo-module
enum class TimeSyncState : uint8_t {
    UNSYNCED = 0,
    SYNCED   = 1,
    COUNT
};

// ---------------------------------------------------------------------------
// stateToString – single source of truth for state-string mapping
// ---------------------------------------------------------------------------
inline const char* stateToString(ShelfbotState s) {
    switch (s) {
        case ShelfbotState::STARTING:  return "starting";
        case ShelfbotState::RUNNING:   return "running";
        case ShelfbotState::ERROR:     return "error";
        case ShelfbotState::SHUTDOWN:  return "shutdown";
        default:                       return "unknown";
    }
}
inline const char* stateToString(MotorControlState s) {
    switch (s) {
        case MotorControlState::OFF:      return "off";
        case MotorControlState::IDLE:     return "idle";
        case MotorControlState::MOVING:   return "moving";
        case MotorControlState::ERROR:    return "error";
        case MotorControlState::DISABLED: return "disabled";
        default:                          return "unknown";
    }
}
inline const char* stateToString(SensorControlState s) {
    switch (s) {
        case SensorControlState::OFF:      return "off";
        case SensorControlState::IDLE:     return "idle";
        case SensorControlState::SCANNING: return "scanning";
        case SensorControlState::ERROR:    return "error";
        case SensorControlState::DISABLED: return "disabled";
        default:                           return "unknown";
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

// ---------------------------------------------------------------------------
// orderedStates – the canonical ordered state vector for each module.
//
// These vectors are passed to StateMachine::setInitial() to enable
// isAtLeast() comparisons.  The order must exactly match the enum value
// order above.
// ---------------------------------------------------------------------------
inline std::vector<std::string> orderedStates(ShelfbotState) {
    return {"starting", "running", "error", "shutdown"};
}
inline std::vector<std::string> orderedStates(MotorControlState) {
    return {"off", "idle", "moving", "error", "disabled"};
}
inline std::vector<std::string> orderedStates(SensorControlState) {
    return {"off", "idle", "scanning", "error", "disabled"};
}
// MicrorosState: DISCONNECTED(1) is intentionally above OFF(0) so that
// the self-prerequisite "microros_sync must be >= disconnected before
// entering discovering" means: can only start discovery from a safe-stopped
// state (disconnected, or any state with higher rank).
inline std::vector<std::string> orderedStates(MicrorosState) {
    return {"off", "disconnected", "discovering", "creating_entities", "connected", "error"};
}
inline std::vector<std::string> orderedStates(WifiManagerState) {
    return {"off", "disconnected", "connecting", "connected", "error"};
}
inline std::vector<std::string> orderedStates(NetworkServiceState) {
    return {"off", "starting", "mdns_ready", "http_running", "error"};
}
inline std::vector<std::string> orderedStates(AgentState) {
    return {"offline", "discovered", "ping_ok", "session_synced",
            "entities_created", "connected", "error"};
}
inline std::vector<std::string> orderedStates(TimeSyncState) {
    return {"unsynced", "synced"};
}
