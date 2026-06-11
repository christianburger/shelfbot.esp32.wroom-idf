#pragma once
#include <idf_c_includes.hpp>

// ---------------------------------------------------------------------------
// StateMachine
//
// Centralises ALL state tracking, all prerequisite enforcement, and all
// transition permission logic for every firmware module.
//
// Rules:
//   1. Every module calls setInitial() once at startup.
//   2. Every state change goes through changeState().
//   3. No module may call getState() on a *different* module to make a
//      control-flow decision.  Use canTransition() or waitForPrerequisites()
//      instead — those are the only sanctioned cross-module state queries.
//   4. Prerequisite relationships are declared once, here, via
//      registerPrerequisites(), and are enforced by canTransition().
// ---------------------------------------------------------------------------
class StateMachine {
public:
    // -----------------------------------------------------------------------
    // Module state record
    // -----------------------------------------------------------------------
    struct ModuleState {
        std::string current_state;
        // Monotonically-increasing rank used for "at-least" comparisons.
        // Assigned by setInitial() based on registration order; callers
        // must provide an ordered list of valid states so that rank can be
        // inferred.  If no rank list is supplied, all states have rank 0 and
        // only exact-match checks are available.
        std::unordered_map<std::string, int> state_rank;

        ModuleState() = default;
        explicit ModuleState(const std::string& state,
                             const std::vector<std::string>& ordered_states = {})
            : current_state(state)
        {
            for (int i = 0; i < static_cast<int>(ordered_states.size()); ++i)
                state_rank[ordered_states[i]] = i;
        }

        int rank(const std::string& s) const {
            auto it = state_rank.find(s);
            return (it != state_rank.end()) ? it->second : 0;
        }
        int current_rank() const { return rank(current_state); }
    };

    // -----------------------------------------------------------------------
    // Prerequisite descriptor
    //
    // "Before module M may enter state T, module P must be at least in state
    //  P_min_state (i.e. have a rank >= rank(P_min_state))."
    // -----------------------------------------------------------------------
    struct Prerequisite {
        std::string prereq_module;      ///< Module that must be ready
        std::string min_state;          ///< Minimum state string (inclusive)
    };

    // -----------------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------------

    /**
     * @brief Initialise the framework and start the periodic status-dump task.
     *        Must be called once before any other StateMachine API.
     */
    static void init();

    // -----------------------------------------------------------------------
    // Module registration
    // -----------------------------------------------------------------------

    /**
     * @brief Register a module with its initial state and an ordered list of
     *        all valid states (lowest rank first).  Must be called exactly once
     *        per module before any transition is attempted.
     *
     * @param module         Module name.
     * @param initial_state  Starting state string.
     * @param ordered_states All valid states in ascending-rank order.
     *                       The initial state must be present in this list.
     *                       Pass {} to skip rank tracking (exact-match only).
     * @return true on success, false if the module already exists.
     */
    static bool setInitial(const std::string& module,
                           const std::string& initial_state,
                           const std::vector<std::string>& ordered_states = {});

    // -----------------------------------------------------------------------
    // Prerequisite registry
    // -----------------------------------------------------------------------

    /**
     * @brief Declare that module @p module may NOT enter state @p target_state
     *        unless @p prereq_module is at-or-above @p min_prereq_state.
     *
     * Multiple prerequisites may be registered for the same (module,
     * target_state) pair; ALL must be satisfied for the transition to be
     * permitted.
     *
     * Must be called after setInitial() for both @p module and
     * @p prereq_module, before any relevant transition is attempted.
     */
    static void registerPrerequisite(const std::string& module,
                                     const std::string& target_state,
                                     const Prerequisite& prereq);

    // -----------------------------------------------------------------------
    // State queries (cross-module use is ONLY permitted through these APIs)
    // -----------------------------------------------------------------------

    /**
     * @brief Returns true if @p module is currently in exactly @p state.
     *        Use sparingly — prefer canTransition() for guard logic.
     */
    static bool isInState(const std::string& module, const std::string& state);

    /**
     * @brief Returns true if @p module's current state has a rank >=
     *        rank(@p min_state).  Requires ordered_states to have been
     *        supplied at setInitial() time.
     */
    static bool isAtLeast(const std::string& module, const std::string& min_state);

    /**
     * @brief Check whether all prerequisites for @p module transitioning to
     *        @p new_state are currently satisfied.
     *
     * Does NOT perform the transition; use changeState() for that.
     * Returns true if there are no registered prerequisites (open permission).
     */
    static bool canTransition(const std::string& module,
                              const std::string& new_state);

    /**
     * @brief Block the calling task until all prerequisites for
     *        (@p module -> @p target_state) are satisfied, or until
     *        @p timeout_ms elapses.
     *
     * Polls every @p poll_ms milliseconds.  Returns true if prerequisites
     * are all met before the timeout, false otherwise.
     *
     * Intended for use inside tasks that must wait before proceeding to a
     * new phase (e.g. microros_sync waiting for Wi-Fi + mDNS + SNTP before
     * attempting entity creation).
     */
    static bool waitForPrerequisites(const std::string& module,
                                     const std::string& target_state,
                                     uint32_t timeout_ms,
                                     uint32_t poll_ms = 200);

    // -----------------------------------------------------------------------
    // State mutation
    // -----------------------------------------------------------------------

    /**
     * @brief Attempt to change @p module's state to @p new_state.
     *
     * The call ALWAYS enforces prerequisite checks unless
     * @p force_skip_prereqs is true (reserved for internal teardown paths
     * that must be able to set "disconnected" even when prerequisites are
     * not met).
     *
     * @return true if the transition was applied, false if the module was
     *         not found, the state is unchanged, or prerequisites failed.
     */
    static bool changeState(const std::string& module,
                            const std::string& new_state,
                            bool force_skip_prereqs = false);

    // -----------------------------------------------------------------------
    // Diagnostic (read-only; do not use for control-flow)
    // -----------------------------------------------------------------------

    /** @brief Get the raw current-state string for a module.
     *         ONLY call this from logging / diagnostic code; never use the
     *         returned value to make a control-flow decision — use
     *         canTransition() or isAtLeast() instead. */
    static const std::string& getState(const std::string& module);

private:
    static std::mutex mutex_;
    static std::unordered_map<std::string, ModuleState> modules_;

    // prerequisites_[module][target_state] = list of Prerequisite
    static std::unordered_map<
        std::string,
        std::unordered_map<std::string, std::vector<Prerequisite>>
    > prerequisites_;

    static TaskHandle_t status_task_handle_;
    static bool         task_running_;

    static void status_dump_task(void* arg);
    static void dumpAllStates();

    // Internal helper: evaluate prerequisites without taking mutex (caller holds it).
    static bool prereqsSatisfied_locked(const std::string& module,
                                        const std::string& new_state);
};
