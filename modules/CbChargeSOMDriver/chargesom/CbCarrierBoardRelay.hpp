// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <ostream>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <sigslot/signal.hpp>
#include "CbRelay.hpp"
#include "CbChargeSOM.hpp"

using namespace std::chrono_literals;

class CbCarrierBoardRelay : public CbRelay {

public:
    enum class Contactor
    {
        Contactor1 = 0,
        Contactor2 = 1,
        Contactor3 = 2,
    };

    /// @brief Constructor
    CbCarrierBoardRelay(CbChargeSOM& controller, std::initializer_list<Contactor> contactors,
                        const std::string& relay_name, bool enslaved = false);

    /// @brief Destructor.
    ~CbCarrierBoardRelay() override;

    /// @brief Tells whether the feedback signal monitoring should be used and actually starts it if so.
    ///        This function must be called before any other member functions are used.
    /// @param use_feedback True - use the feedback signal; false - ignore it.
    /// @param active_low Tells whether the feedback line should use inverted logic.
    void start(bool use_feedback, bool active_low) override;

    /// @brief Switch the relay on/off.
    ///        Usually, the function waits for the feedback confirming the switch, but this can be prevented
    ///        since in some setups, we might not see the feedback immediately.
    /// @param on The target state (true = on, false = off)
    /// @param wait_for_feedback Tell whether to wait and evaluate the feedback before returning.
    /// @return Returns false if the new state could not reached successfully (based on sense signal evaluation
    ///         if configured and thus it is probably a contactor error), true otherwise.
    bool set_actuator_state(bool on, bool wait_for_feedback) override;

    /// @brief Read the actual state of the actuator.
    bool get_actuator_state() const override;

    /// @brief Read the actual GPIO state of the feedback. If `start` was called with `use_feedback` set to `false`,
    ///        then this function just returns the current actuator state.
    bool get_feedback_state() const override;

    /// @brief Tell the feedback monitor about an expected feedback change which is not initiated by
    ///        a call to `set_actuator_state`, e.g. because of a special serial wiring of multiple contactors.
    void set_expected_feedback_change(bool on) override;

    /// @brief Sleeps until the next change event on the feedback signal occurs.
    /// @return Returns false if the new state could not reached successfully (based on sense signal evaluation
    ///         if configured and thus it is probably a contactor error), true otherwise.
    bool wait_for_feedback() override;

    /// @brief Return the time to wait until the relay can be closed again.
    std::chrono::milliseconds get_closing_delay_left() const override;

private:
    /// @brief Remember the reference to the controller instance
    CbChargeSOM& controller;

    /// @brief Name of the relay and its feedback as labeled on hardware.
    std::string relay_name;

    /// @brief Determines whether we can make a sync call to the controller or not.
    bool is_enslaved {false};

    /// @brief Remember which contactors whether we can make a sync call to the controller or not.
    std::set<unsigned int> contactor_indexes;

    /// @brief Flag whether the feedback is used.
    bool has_feedback {false};

    /// @brief Remember the next expected feedback edge event type in case of an actuator change initiated by ourself.
    std::optional<types::cb_board_support::ContactorState> expected_edge;

    /// @brief Set the next expected feedback edge event type
    ///        Must be called with the mutex `expected_edge_mutex` hold.
    void set_expected_edge(bool on);

    /// @brief Sleeps until the next change event on the feedback signal occurs.
    /// @param lock Reference to the lock to re-use.
    /// @return Returns false if the new state could not reached successfully (based on sense signal evaluation
    ///         if configured and thus it is probably a contactor error), true otherwise.
    bool wait_for_feedback_lock(std::unique_lock<std::mutex>& lock);

    /// @brief This is basically the return value of `set_actuator_state`.
    std::optional<bool> expected_edge_matched;

    /// @brief Protects `expected_edge` and `expected_edge_matched.
    std::mutex expected_edge_mutex;

    /// @brief Used to signal `expected_edge_matched` back to the `set_actuator_state` method.
    std::condition_variable cv_expected_edge;

    /// @brief The time in milliseconds until the feedback should have reported
    ///        an edge event when the relay was switched.
    std::chrono::milliseconds feedback_timeout {500ms};

    std::ostream& dump(std::ostream& os) const override;
};
