#pragma once

#include <idf_c_includes.hpp>
class StateMachine {
public:
    static bool init();
    static bool setInitial(const std::string& module_name, const std::string& initial_state);
    static bool changeState(const std::string& module_name, const std::string& new_state);
    static std::string getCurrentState(const std::string& module_name);
    static int64_t getCurrentStateTimestamp(const std::string& module_name);
    static std::string getPreviousState(const std::string& module_name);
    static int64_t getPreviousStateTimestamp(const std::string& module_name);
    static void logCurrentState(const std::string& module_name);

private:
    struct ModuleState {
        std::string current_state_str;
        int64_t     current_timestamp;
        std::string previous_state_str;
        int64_t     previous_timestamp;
    };
    static std::unordered_map<std::string, ModuleState> modules_;
    static std::mutex mutex_;
    static void updateState(const std::string& module_name, const std::string& new_state_str);
};