// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <ostream>
#include <optional>
#include <string>
#include <thread>
#include <sigslot/signal.hpp>
#include <gpiod.hpp>
#include "CbActuator.hpp"

using namespace std::chrono_literals;

class CbTarragonRelay {

public:
    /// @brief Constructor
    /// @param relay_name Name of the relay and its feedback sense as labeled on hardware.
    /// @param actuator_ref The actuator reference to control the relais coil.
    /// @param feedback_gpio_line_name The name of the GPIO line to which the feedback (aka sense) circuit is
    /// connected to.
    /// @param feedback_gpio_debounce_us The debounce period which should be configured for the feedback GPIO (in [us]).
    /// Pass a value of zero to skip any configuration.
    CbTarragonRelay(const std::string& relay_name, const CbActuatorReference actuator_ref,
                    const std::string& feedback_gpio_line_name, const unsigned int feedback_gpio_debounce_us);

    /// @brief Destructor.
    ~CbTarragonRelay();

    /// @brief Tells whether the feedback signal monitoring should be used and actually starts it if so.
    ///        This function must be called before any other member functions are used.
    /// @param use_feedback True - use the feedback signal; false - ignore it.
    /// @param active_low Tells whether the feedback line should use inverted logic.
    void start(bool use_feedback, bool active_low);

    /// @brief Switch the relay on/off. This function ensures that the minimum close interval between
    ///        consecutive closings is respected (ie. waits until the GPIO can be actually switched).
    ///        Usually, the function waits for the feedback confirming the switch, but this can be prevented
    ///        since in some setups, we might not see the feedback immediately.
    /// @param on The target state (true = on, false = off)
    /// @param wait_for_feedback Tell whether to wait and evaluate the feedback before returning.
    /// @return Returns false if the new state could not reached successfully (based on sense signal evaluation
    ///         if configured and thus it is probably a contactor error), true otherwise.
    bool set_actuator_state(bool on, bool wait_for_feedback);

    /// @brief Read the actual GPIO state of the actuator.
    bool get_actuator_state() const;

    /// @brief Read the actual GPIO state of the feedback. If `start` was called with `use_feedback` set to `false`,
    ///        then this function just returns the current actuator state.
    bool get_feedback_state() const;

    /// @brief Tell the feedback monitor about an expected feedback change which is not initiated by
    ///        a call to `set_actuator_state`, e.g. because of a special serial wiring of multiple contactors.
    void set_expected_feedback_change(bool on);

    /// @brief Sleeps until the next change event on the feedback signal occurs.
    /// @return Returns false if the new state could not reached successfully (based on sense signal evaluation
    ///         if configured and thus it is probably a contactor error), true otherwise.
    bool wait_for_feedback();

    /// @brief Return the time to wait until the relay can be closed again.
    std::chrono::milliseconds get_closing_delay_left() const;

    /// @brief Signal used to inform about unexpected state changes on the feedback GPIO.
    sigslot::signal<const std::string&, bool> on_unexpected_change;

    /// @brief Feeds a string representation of the given CbTarragonRelay instance into an output stream.
    /// @return A reference to the output stream operated on.
    friend std::ostream& operator<<(std::ostream& os, const CbTarragonRelay& r);

private:
    /// @brief Name of the relay and its feedback as labeled on hardware.
    std::string relay_name;

    /// @brief The reference to the used actuator.
    CbActuatorReference actuator_ref;

    /// @brief Name of the GPIO line to use for feedback/sense detection.
    std::string feedback_gpio_line_name;

    /// @brief The debounce period which should be configured for the feedback GPIO (in [us]).
    const unsigned int feedback_gpio_debounce_us;

    /// @brief The GPIO handle of the used feedback (RELAY_1_SENSE or RELAY_2_SENSE).
    std::unique_ptr<gpiod::line_request> feedback;

    /// @brief This helper allows to remember whether a `set_actuator_state(false)` was
    ///        seen while we wait for the `min_close_interval`.
    std::atomic_bool execute_actuator_state_on_switch {false};

    /// @brief Remember the next expected feedback edge event type in case of an actuator change initiated by ourself.
    std::optional<gpiod::edge_event::event_type> expected_edge;

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

    /// @brief The timestamp which records the moment when the contactor's state transitioned
    ///        from OPEN to CLOSED.
    std::chrono::time_point<std::chrono::steady_clock> last_closed_ts;

    /// @brief The time in seconds which represents a minimum duration between relay
    ///        closings to prevent internal relay wear out. The worst considered relay type
    //         can withstand a maximum of 6 cycles (open and close actions) per
    ///        minute i.e., 10 seconds between 2 relay closings.
    std::chrono::milliseconds min_close_interval {10s};

    /// @brief The time in milliseconds until the feedback GPIO should have reported
    ///        an edge event when the relay was switched.
    std::chrono::milliseconds feedback_timeout {250ms};

    /// @brief Helper to signal thread termination wish
    std::atomic_bool termination_requested {false};

    /// @brief Thread handle of feedback GPIO monitoring thread.
    std::thread feedback_monitor;

    /// @brief Feedback GPIO monitoring thread worker function.
    void feedback_monitor_worker();
};
