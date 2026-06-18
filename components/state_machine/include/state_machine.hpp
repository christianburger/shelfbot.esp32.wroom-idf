#pragma once

#include <idf_c_includes.hpp>
#include <unordered_map>
#include <vector>
#include <string>

// ---------------------------------------------------------------------------
// StateMachine
//
// Locking model
// -------------
// All public methods attempt to take the internal mutex with a 100 ms timeout.
// If the mutex cannot be acquired within that time, the method logs an error
// and returns a safe default (false, empty string, or does nothing).
// This prevents indefinite blocking and makes the system robust against
// accidental long holds by other tasks.
//
// Internal helpers suffixed with _locked must only be called while the
// caller holds the mutex.
// ---------------------------------------------------------------------------

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

    // Read-only queries — all self-locking with timeout.
    static bool isAtLeast(const std::string& module, const std::string& min_state);
    static bool isInState(const std::string& module, const std::string& state);
    static bool canTransition(const std::string& module, const std::string& new_state);
    static bool waitForPrerequisites(const std::string& module,
                                     const std::string& target_state,
                                     uint32_t timeout_ms,
                                     uint32_t poll_ms = 250);
    static const std::string& getState(const std::string& module);

    // Orchestration — self-locking with timeout.
    static void advance();                          // progress all modules (void, not used)
    static bool advance(const std::string& module); // progress one module, returns true if transitioned
    static void recover();                          // reset error/recovery states

    // Direct state change — self-locking with timeout.
    // Safe to call from any component without holding the mutex.
    // Use force_skip_prereqs=true only for components that own their state
    // (e.g. wifi_manager reporting hardware events).
    static bool changeState(const std::string& module,
                            const std::string& new_state,
                            bool force_skip_prereqs = false);

private:
    static SemaphoreHandle_t mutex_;
    static std::unordered_map<std::string, ModuleState> modules_;
    static std::unordered_map<
        std::string,
        std::unordered_map<std::string, std::vector<Prerequisite>>
    > prerequisites_;

    static TaskHandle_t status_task_handle_;
    static bool         task_running_;

    // ── Internal helpers (caller must hold mutex_) ────────────────────────────
    static bool prereqsSatisfied_locked(const std::string& module,
                                        const std::string& new_state);

    // changeState_locked: write path used internally by advance() and recover()
    // which already hold mutex_.  Identical logic to changeState() but without
    // acquiring the lock a second time.
    static bool changeState_locked(const std::string& module,
                                   const std::string& new_state,
                                   bool force_skip_prereqs = false);

    static void status_dump_task(void* arg);
    static void dumpAllStates();

    // Helper to safely take the mutex with a timeout.
    static bool takeMutex();
    static void giveMutex();
};