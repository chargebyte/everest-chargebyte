// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once
#include <chrono>
#include <sstream>
#include <string>
#include <sigslot/signal.hpp>
#include <generated/types/cb_board_support.hpp>
#include "CbTarragonContactor.hpp"

///
/// This is an "abstract" (not in sense of C++ meaning) class for handling contactor control.
/// It only defines the interface which is used by the upper layer.
/// Derived classes implement specific contactor handling setups.
///
class CbTarragonContactorControl {

public:
    /// @brief Constructor.
    CbTarragonContactorControl() {};

    /// @brief Checks the actual states of actuator and feedback for plausibility.
    ///        This is intended to be called once during startup only, since
    ///        error reporting is limited during this phase.
    /// @param[out] error_hint A string with a user hint which looks unexpected.
    /// @return True in case there is an unexpected state detected, false otherwise.
    virtual bool is_inconsistent_state(std::ostringstream& error_hint) const = 0;

    /// @brief Closes the contactor (on = true), or opens it (on = false):
    ///        This is a synchronous call, i.e. it waits until it is confirmed
    ///        by feedback signal (if used). It also waits the required time
    ///        in case the relay is protected by a maximum switching time to
    ///        prevent wear out.
    /// @return True on success, false on error.
    virtual bool switch_state(bool on) = 0;

    /// @brief Opens the contactor. Convenience helper just for code readability.
    /// @return True on success, false on error.
    bool open();

    /// @brief Return the current state of the contactor based on the actuator line.
    /// @return True when contactor is closed, false otherwise.
    virtual bool get_state() const = 0;

    /// @brief Return the time to wait until the relay can be closed again.
    virtual std::chrono::milliseconds get_closing_delay_left() const = 0;

    /// @brief Set the desired count of phases for next contactor closing.
    void switch_phase_count(bool use_3phases);

    /// @brief Signal used to inform about errors during switching.
    sigslot::signal<const std::string&, bool, types::cb_board_support::ContactorState> on_error;

    /// @brief Signal used to inform about unexpected state changes on the feedback GPIO(s).
    sigslot::signal<const std::string&, types::cb_board_support::ContactorState> on_unexpected_change;

    /// @brief Feeds a string representation of the given CbTarragonContactorControl
    ///        instance into an output stream.
    /// @return A reference to the output stream operated on.
    friend std::ostream& operator<<(std::ostream& os, const CbTarragonContactorControl& c);

protected:
    /// @brief Desired number of phases (1 or 3).
    int phase_count {3};

    /// @brief Helper which switches the given contactor and reports an error if
    ///        the switching failed.
    /// @param contactor Reference to the contactor instance to operate on.
    /// @param on The target state (true = on, false = off)
    /// @param wait_for_feedback Tell whether to wait and evaluate the feedback before returning.
    /// @return True on success, false on error.
    bool switch_contactor(CbTarragonContactor& contactor, bool on, bool wait_for_feedback = true);

    /// @brief Helper to feed a string representation into an output stream.
    /// @param os Output stream reference to operate on.
    /// @return A reference to the output stream operated on.
    virtual std::ostream& dump(std::ostream& os) const;
};
