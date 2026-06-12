#pragma once
#include <idf_c_includes.hpp>

// ---------------------------------------------------------------------------
// StateMachine — single source of truth for all module states
//
// Contract:
//   1.  setInitial()           – call ONCE per module at initialisation time.
//   2.  registerPrerequisite() – declare cross-module dependencies.
//   3.  changeState()          – the ONLY way to mutate state.
//   4.  canTransition()        – check prereqs without mutating.
//   5.  waitForPrerequisites() – block until prereqs are met or timeout.
//   6.  isAtLeast() / isInState() – safe read-only queries from timers/tasks.
//   7.  getState()             – diagnostic / logging only; never use the
//                                returned value for a control-flow decision.
//
// No module may maintain its own parallel state variable for the same
// information tracked here.
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
    //
    //   "module M must not enter target_state unless
    //    prereq_module is at rank >= rank(min_state)"
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

    /** Register a module. ordered_states must be supplied if any isAtLeast()
     *  or rank-based prereq checks will be used for this module.
     *  Returns false (and logs a warning) if the module already exists. */
    static bool setInitial(const std::string& module,
                           const std::string& initial_state,
                           const std::vector<std::string>& ordered_states = {});

    /** Declare a cross-module dependency.  Call from init-time code, BEFORE
     *  start() is called, so all prerequisites exist when the task runs. */
    static void registerPrerequisite(const std::string& module,
                                     const std::string& target_state,
                                     const Prerequisite& prereq);

    // -----------------------------------------------------------------------
    // Cross-module state queries — use ONLY these for control-flow
    // -----------------------------------------------------------------------

    /** True if module's current state rank >= rank(min_state). */
    static bool isAtLeast(const std::string& module,
                          const std::string& min_state);

    /** True if module is in exactly the given state. */
    static bool isInState(const std::string& module,
                          const std::string& state);

    /** True if all registered prerequisites for (module -> new_state) are
     *  currently satisfied.  Does not mutate any state. */
    static bool canTransition(const std::string& module,
                              const std::string& new_state);

    /** Block until all prerequisites for (module -> target_state) are met, or
     *  until timeout_ms elapses.  Polls every poll_ms milliseconds.
     *  Returns true on success, false on timeout. */
    static bool waitForPrerequisites(const std::string& module,
                                     const std::string& target_state,
                                     uint32_t timeout_ms,
                                     uint32_t poll_ms = 250);

    // -----------------------------------------------------------------------
    // State mutation
    // -----------------------------------------------------------------------

    /** Change state, enforcing prerequisites unless force_skip_prereqs=true.
     *  force_skip_prereqs is ONLY for teardown paths (going to "disconnected"
     *  or "off" from any state, regardless of what other modules are doing).
     *  Returns false if module not found, state unchanged, or prereqs failed. */
    static bool changeState(const std::string& module,
                            const std::string& new_state,
                            bool force_skip_prereqs = false);

    // -----------------------------------------------------------------------
    // Diagnostic only — do NOT use for control flow
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

    // Must be called with mutex_ already held.
    static bool prereqsSatisfied_locked(const std::string& module,
                                        const std::string& new_state);

    static void status_dump_task(void* arg);
    static void dumpAllStates();
};
