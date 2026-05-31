#pragma once
#include <idf_c_includes.hpp>

class StateMachine {
public:
    struct ModuleState {
        std::string current_state;
        ModuleState() = default;
        explicit ModuleState(const std::string& state) : current_state(state) {}
    };

    /**
     * @brief Initialise the state machine and start the periodic status logging task.
     * Must be called once from app_main() before any other StateMachine calls.
     */
    static void init();

    /**
     * @brief Set the initial state for a module.
     * @param module Name of the module (e.g., "microros_sync", "sensor_control").
     * @param initial_state Initial state string.
     * @return true if state was set, false if module already exists.
     */
    static bool setInitial(const std::string& module, const std::string& initial_state);

    /**
     * @brief Change the state of an existing module.
     * @param module Name of the module.
     * @param new_state New state string.
     * @return true if state changed, false if module not found.
     */
    static bool changeState(const std::string& module, const std::string& new_state);

    /**
     * @brief Get the current state of a module.
     * @param module Name of the module.
     * @return Reference to state string, or empty string if module not found.
     */
    static const std::string& getState(const std::string& module);

private:
    static std::mutex mutex_;                                   // Protects modules_
    static std::unordered_map<std::string, ModuleState> modules_;

    static TaskHandle_t status_task_handle_;
    static bool task_running_;

    /**
     * @brief FreeRTOS task that logs all module states every 10 seconds.
     */
    static void status_dump_task(void* arg);

    /**
     * @brief Internal helper to dump current states (called from the task).
     */
    static void dumpAllStates();
};