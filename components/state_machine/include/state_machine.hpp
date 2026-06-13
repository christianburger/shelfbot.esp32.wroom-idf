#pragma once

#include <idf_c_includes.hpp>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <string>

class StateMachine {
public:
    struct ModuleState {
        std::string current_state;
        std::unordered_map<std::string, int> state_rank;

        ModuleState() = default;
        explicit ModuleState(const std::string& initial,
                             const std::vector<std::string>& ordered)
            : current_state(initial)
        {
            for (int i = 0; i < static_cast<int>(ordered.size()); ++i)
                state_rank[ordered[i]] = i;
        }

        int rank(const std::string& s) const {
            auto it = state_rank.find(s);
            return (it != state_rank.end()) ? it->second : -1;
        }
        int current_rank() const { return rank(current_state); }
    };

    struct Prerequisite {
        std::string prereq_module;
        std::string min_state;
    };

    // Lifecycle & registration
    static void init();
    static bool setInitial(const std::string& module,
                           const std::string& initial_state,
                           const std::vector<std::string>& ordered_states = {});
    static void registerPrerequisite(const std::string& module,
                                     const std::string& target_state,
                                     const Prerequisite& prereq);

    // Read‑only queries
    static bool isAtLeast(const std::string& module, const std::string& min_state);
    static bool isInState(const std::string& module, const std::string& state);
    static bool canTransition(const std::string& module, const std::string& new_state);
    static bool waitForPrerequisites(const std::string& module,
                                     const std::string& target_state,
                                     uint32_t timeout_ms,
                                     uint32_t poll_ms = 250);
    static const std::string& getState(const std::string& module);

    // Orchestration – preferred way to progress most modules
    static void advance();                     // progress all modules
    static void advance(const std::string& module); // progress one module
    static void recover();                     // reset error/recovery states

    // Direct state change – allowed ONLY for modules that own their state
    // (e.g., wifi_manager reporting WiFi hardware state). Other components
    // should use advance() or recover().
    static bool changeState(const std::string& module,
                            const std::string& new_state,
                            bool force_skip_prereqs = false);

private:
    static std::mutex mutex_;
    static std::unordered_map<std::string, ModuleState> modules_;
    static std::unordered_map<
        std::string,
        std::unordered_map<std::string, std::vector<Prerequisite>>
    > prerequisites_;

    static TaskHandle_t status_task_handle_;
    static bool         task_running_;

    static bool prereqsSatisfied_locked(const std::string& module,
                                        const std::string& new_state);

    static void status_dump_task(void* arg);
    static void dumpAllStates();
};