#pragma once
#include <idf_c_includes.hpp>

// ----------------------------------------------------------------------------
// Module state enumerations
// ----------------------------------------------------------------------------
enum class ShelfbotState : uint8_t {
    STARTING = 0,
    RUNNING,
    ERROR,
    SHUTDOWN,
    COUNT
};

enum class MotorControlState : uint8_t {
    OFF = 0,
    IDLE,
    MOVING,
    ERROR,
    DISABLED,
    COUNT
};

enum class SensorControlState : uint8_t {
    OFF = 0,
    IDLE,
    SCANNING,
    ERROR,
    DISABLED,
    COUNT
};

enum class MicrorosState : uint8_t {
    OFF = 0,
    DISCOVERING,
    TIME_SYNC,      ///< Agent reachable; waiting for clock synchronisation
    CONNECTED,
    ERROR,
    DISCONNECTED,
    COUNT
};

enum class WifiManagerState : uint8_t {
    OFF = 0,
    CONNECTING,
    CONNECTED,
    ERROR,
    DISCONNECTED,
    COUNT
};

// New state for the network services task (mDNS + HTTP)
enum class NetworkServiceState : uint8_t {
    OFF = 0,
    STARTING,
    MDNS_READY,
    HTTP_RUNNING,
    ERROR,
    COUNT
};

// ----------------------------------------------------------------------------
// Convert enum to string (for logging and state machine key)
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
        case MicrorosState::OFF:          return "off";
        case MicrorosState::DISCOVERING:  return "discovering";
        case MicrorosState::TIME_SYNC:    return "time_sync";
        case MicrorosState::CONNECTED:    return "connected";
        case MicrorosState::ERROR:        return "error";
        case MicrorosState::DISCONNECTED: return "disconnected";
        default:                          return "unknown";
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

// ----------------------------------------------------------------------------
// Transition matrices (informational — StateMachine does not enforce them yet)
// ----------------------------------------------------------------------------

// Shelfbot (4 states: STARTING, RUNNING, ERROR, SHUTDOWN)
constexpr std::array<
    std::array<bool, static_cast<size_t>(ShelfbotState::COUNT)>,
    static_cast<size_t>(ShelfbotState::COUNT)>
shelfbot_transitions = {{
    /* STARTING */ {{ false, true,  true,  false }},
    /* RUNNING  */ {{ false, false, true,  true  }},
    /* ERROR    */ {{ false, false, false, true  }},
    /* SHUTDOWN */ {{ false, false, false, false }}
}};

// MotorControl (5 states: OFF, IDLE, MOVING, ERROR, DISABLED)
constexpr std::array<
    std::array<bool, static_cast<size_t>(MotorControlState::COUNT)>,
    static_cast<size_t>(MotorControlState::COUNT)>
motor_control_transitions = {{
    /* OFF      */ {{ false, true,  false, false, false }},
    /* IDLE     */ {{ true,  false, true,  true,  false }},
    /* MOVING   */ {{ false, true,  false, true,  false }},
    /* ERROR    */ {{ false, false, false, false, true  }},
    /* DISABLED */ {{ false, false, false, false, false }}
}};

// SensorControl (5 states: OFF, IDLE, SCANNING, ERROR, DISABLED)
constexpr std::array<
    std::array<bool, static_cast<size_t>(SensorControlState::COUNT)>,
    static_cast<size_t>(SensorControlState::COUNT)>
sensor_control_transitions = {{
    /* OFF      */ {{ false, true,  false, false, false }},
    /* IDLE     */ {{ true,  false, true,  true,  false }},
    /* SCANNING */ {{ false, true,  false, true,  false }},
    /* ERROR    */ {{ false, false, false, false, true  }},
    /* DISABLED */ {{ false, false, false, false, false }}
}};

// MicrorosSync (6 states: OFF, DISCOVERING, TIME_SYNC, CONNECTED, ERROR, DISCONNECTED)
constexpr std::array<
    std::array<bool, static_cast<size_t>(MicrorosState::COUNT)>,
    static_cast<size_t>(MicrorosState::COUNT)>
microros_transitions = {{
    /* OFF          */ {{ false, true,  false, false, false, false }},
    /* DISCOVERING  */ {{ false, false, true,  true,  true,  true  }},
    /* TIME_SYNC    */ {{ false, false, false, true,  true,  true  }},
    /* CONNECTED    */ {{ false, false, false, false, true,  true  }},
    /* ERROR        */ {{ true,  true,  false, false, false, false }},
    /* DISCONNECTED */ {{ false, true,  false, false, false, false }}
}};

// WifiManager (5 states: OFF, CONNECTING, CONNECTED, ERROR, DISCONNECTED)
constexpr std::array<
    std::array<bool, static_cast<size_t>(WifiManagerState::COUNT)>,
    static_cast<size_t>(WifiManagerState::COUNT)>
wifi_manager_transitions = {{
    /* OFF          */ {{ false, true,  false, false, false }},
    /* CONNECTING   */ {{ false, false, true,  true,  true  }},
    /* CONNECTED    */ {{ false, false, false, true,  true  }},
    /* ERROR        */ {{ true,  false, false, false, false }},
    /* DISCONNECTED */ {{ false, true,  false, false, false }}
}};

// NetworkService (5 states: OFF, STARTING, MDNS_READY, HTTP_RUNNING, ERROR)
constexpr std::array<
    std::array<bool, static_cast<size_t>(NetworkServiceState::COUNT)>,
    static_cast<size_t>(NetworkServiceState::COUNT)>
network_service_transitions = {{
    /* OFF         */ {{ false, true,  false, false, false }},
    /* STARTING    */ {{ false, false, true,  true,  true  }},
    /* MDNS_READY  */ {{ false, false, false, true,  true  }},
    /* HTTP_RUNNING*/ {{ false, false, false, false, true  }},
    /* ERROR       */ {{ false, false, false, false, false }}
}};