#pragma once

#include <idf_c_includes.hpp>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <string>

// ---------------------------------------------------------------------------
// StateMachine — single source of truth for all module states
//
// Contract:
//   1.  setInitial()           – call ONCE per module at initialisation time.
//   2.  registerPrerequisite() – declare cross-module dependencies.
//   3.  changeState()          – the ONLY way to mutate state.
//   4.  canTransition()        – check prereqs without mutating.
//   5.  waitForPrerequisites() – block until prereqs are met or timeout.
//   6.  isAtLeast() / isInState() – safe read-only queries.
//   7.  getState()             – diagnostic only.
// ---------------------------------------------------------------------------
class StateMachine {
public:
    // -----------------------------------------------------------------------
    // Module state record
    // -----------------------------------------------------------------------
    struct ModuleState {
        std::string                        current_state;
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

    // -----------------------------------------------------------------------
    // Prerequisite descriptor
    // -----------------------------------------------------------------------
    struct Prerequisite {
        std::string prereq_module;
        std::string min_state;
    };

    // -----------------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------------
    static void init();

    // -----------------------------------------------------------------------
    // Registration
    // -----------------------------------------------------------------------
    static bool setInitial(const std::string& module,
                           const std::string& initial_state,
                           const std::vector<std::string>& ordered_states = {});

    static void registerPrerequisite(const std::string& module,
                                     const std::string& target_state,
                                     const Prerequisite& prereq);

    // -----------------------------------------------------------------------
    // Cross-module state queries
    // -----------------------------------------------------------------------
    static bool isAtLeast(const std::string& module,
                          const std::string& min_state);

    static bool isInState(const std::string& module,
                          const std::string& state);

    static bool canTransition(const std::string& module,
                              const std::string& new_state);

    static bool waitForPrerequisites(const std::string& module,
                                     const std::string& target_state,
                                     uint32_t timeout_ms,
                                     uint32_t poll_ms = 250);

    // -----------------------------------------------------------------------
    // State mutation
    // -----------------------------------------------------------------------
    static bool changeState(const std::string& module,
                            const std::string& new_state,
                            bool force_skip_prereqs = false);

    // -----------------------------------------------------------------------
    // Diagnostic only
    // -----------------------------------------------------------------------
    static const std::string& getState(const std::string& module);

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