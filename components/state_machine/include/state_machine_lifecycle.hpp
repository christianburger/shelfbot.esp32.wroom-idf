#pragma once
#include <idf_c_includes.hpp>

enum class ShelfbotState : uint8_t { STARTING = 0, RUNNING, ERROR, SHUTDOWN, COUNT };
enum class MotorControlState : uint8_t { OFF = 0, IDLE, MOVING, ERROR, DISABLED, COUNT };
enum class SensorControlState : uint8_t { OFF = 0, IDLE, SCANNING, ERROR, DISABLED, COUNT };
enum class MicrorosState : uint8_t {
    OFF = 0,
    DISCOVERING,
    TIME_SYNC,
    CREATING_ENTITIES,
    CONNECTED,
    ERROR,
    DISCONNECTED,
    COUNT
};
enum class WifiManagerState : uint8_t { OFF = 0, CONNECTING, CONNECTED, ERROR, DISCONNECTED, COUNT };
enum class NetworkServiceState : uint8_t { OFF = 0, STARTING, MDNS_READY, HTTP_RUNNING, ERROR, COUNT };

// New: micro-ROS agent states (remote host)
enum class AgentState : uint8_t {
    OFFLINE = 0,
    DISCOVERED,
    PING_OK,
    SESSION_SYNCED,
    ENTITIES_CREATED,
    CONNECTED,
    ERROR,
    COUNT
};

// ----------------------------------------------------------------------------
// Convert enums to string (for state machine keys)
// ----------------------------------------------------------------------------
inline const char* stateToString(ShelfbotState s) {
    switch(s) {
        case ShelfbotState::STARTING:  return "starting";
        case ShelfbotState::RUNNING:   return "running";
        case ShelfbotState::ERROR:     return "error";
        case ShelfbotState::SHUTDOWN:  return "shutdown";
        default:                       return "unknown";
    }
}

inline const char* stateToString(MotorControlState s) {
    switch(s) {
        case MotorControlState::OFF:      return "off";
        case MotorControlState::IDLE:     return "idle";
        case MotorControlState::MOVING:   return "moving";
        case MotorControlState::ERROR:    return "error";
        case MotorControlState::DISABLED: return "disabled";
        default:                          return "unknown";
    }
}

inline const char* stateToString(SensorControlState s) {
    switch(s) {
        case SensorControlState::OFF:      return "off";
        case SensorControlState::IDLE:     return "idle";
        case SensorControlState::SCANNING: return "scanning";
        case SensorControlState::ERROR:    return "error";
        case SensorControlState::DISABLED: return "disabled";
        default:                           return "unknown";
    }
}

inline const char* stateToString(MicrorosState s) {
    switch(s) {
        case MicrorosState::OFF:              return "off";
        case MicrorosState::DISCOVERING:      return "discovering";
        case MicrorosState::TIME_SYNC:        return "time_sync";
        case MicrorosState::CREATING_ENTITIES:return "creating_entities";
        case MicrorosState::CONNECTED:        return "connected";
        case MicrorosState::ERROR:            return "error";
        case MicrorosState::DISCONNECTED:     return "disconnected";
        default:                              return "unknown";
    }
}

inline const char* stateToString(WifiManagerState s) {
    switch(s) {
        case WifiManagerState::OFF:          return "off";
        case WifiManagerState::CONNECTING:   return "connecting";
        case WifiManagerState::CONNECTED:    return "connected";
        case WifiManagerState::ERROR:        return "error";
        case WifiManagerState::DISCONNECTED: return "disconnected";
        default:                             return "unknown";
    }
}

inline const char* stateToString(NetworkServiceState s) {
    switch(s) {
        case NetworkServiceState::OFF:          return "off";
        case NetworkServiceState::STARTING:     return "starting";
        case NetworkServiceState::MDNS_READY:   return "mdns_ready";
        case NetworkServiceState::HTTP_RUNNING: return "http_running";
        case NetworkServiceState::ERROR:        return "error";
        default:                                return "unknown";
    }
}

inline const char* stateToString(AgentState s) {
    switch(s) {
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