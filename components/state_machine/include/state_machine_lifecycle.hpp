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

// ----------------------------------------------------------------------------
// Convert enum to string (for logging / API)
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

// ----------------------------------------------------------------------------
// Transition matrices (N x N, where N = COUNT of states)
// ----------------------------------------------------------------------------
// Shelfbot (4 states)
constexpr std::array<std::array<bool, static_cast<size_t>(ShelfbotState::COUNT)>, static_cast<size_t>(ShelfbotState::COUNT)> shelfbot_transitions = {{
    /* FROM STARTING */    {{ false, true,  true,  false }}, // to RUNNING, ERROR
    /* FROM RUNNING */     {{ false, false, true,  true  }}, // to ERROR, SHUTDOWN
    /* FROM ERROR */       {{ false, false, false, true  }}, // to SHUTDOWN
    /* FROM SHUTDOWN */    {{ false, false, false, false }}  // terminal
}};

// MotorControl (5 states)
constexpr std::array<std::array<bool, static_cast<size_t>(MotorControlState::COUNT)>, static_cast<size_t>(MotorControlState::COUNT)> motor_control_transitions = {{
    /* OFF     */ {{ false, true,  false, false, false }}, // to IDLE
    /* IDLE    */ {{ true,  false, true,  true,  false }}, // to OFF, MOVING, ERROR
    /* MOVING  */ {{ false, true,  false, true,  false }}, // to IDLE, ERROR
    /* ERROR   */ {{ false, false, false, false, true  }}, // to DISABLED
    /* DISABLED*/ {{ false, false, false, false, false }}
}};

// SensorControl (5 states)
constexpr std::array<std::array<bool, static_cast<size_t>(SensorControlState::COUNT)>, static_cast<size_t>(SensorControlState::COUNT)> sensor_control_transitions = {{
    /* OFF     */ {{ false, true,  false, false, false }}, // to IDLE
    /* IDLE    */ {{ true,  false, true,  true,  false }}, // to OFF, SCANNING, ERROR
    /* SCANNING*/ {{ false, true,  false, true,  false }}, // to IDLE, ERROR
    /* ERROR   */ {{ false, false, false, false, true  }}, // to DISABLED
    /* DISABLED*/ {{ false, false, false, false, false }}
}};

// Microros (5 states)
constexpr std::array<std::array<bool, static_cast<size_t>(MicrorosState::COUNT)>, static_cast<size_t>(MicrorosState::COUNT)> microros_transitions = {{
    /* OFF         */ {{ false, true,  false, false, false }}, // to DISCOVERING
    /* DISCOVERING */ {{ false, false, true,  true,  true  }}, // to CONNECTED, ERROR, DISCONNECTED
    /* CONNECTED   */ {{ false, false, false, true,  true  }}, // to ERROR, DISCONNECTED
    /* ERROR       */ {{ true,  true,  false, false, false }}, // to OFF, DISCOVERING
    /* DISCONNECTED*/ {{ false, true,  false, false, false }}  // to DISCOVERING
}};

// WifiManager (5 states)
constexpr std::array<std::array<bool, static_cast<size_t>(WifiManagerState::COUNT)>, static_cast<size_t>(WifiManagerState::COUNT)> wifi_manager_transitions = {{
    /* OFF         */ {{ false, true,  false, false, false }}, // to CONNECTING
    /* CONNECTING  */ {{ false, false, true,  true,  true  }}, // to CONNECTED, ERROR, DISCONNECTED
    /* CONNECTED   */ {{ false, false, false, true,  true  }}, // to ERROR, DISCONNECTED
    /* ERROR       */ {{ true,  false, false, false, false }}, // to OFF
    /* DISCONNECTED*/ {{ false, true,  false, false, false }}  // to CONNECTING
}};