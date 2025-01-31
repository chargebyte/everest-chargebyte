// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <sigslot/signal.hpp>
#include <generated/types/cb_board_support.hpp>
#include "CbTarragonRelay.hpp"
#include "CbTarragonContactor.hpp"

class CbTarragonContactor {

public:
    /// @brief Constructor.
    /// @param name An description, eg. a simple "Contactor" or "Primary contactor"...
    /// @param relay A pointer to an instance of CbTarragonRelay.
    /// @param contactor_feedback_type Defines the logic behind the feedback (no = normally open, nc = normally close,
    /// none = no feedback).
    CbTarragonContactor(const std::string& name, std::unique_ptr<CbTarragonRelay> relay,
                        const std::string& contactor_feedback_type);

    /// @brief Returns the contactor name
    /// @return The contactor name
    const std::string& get_name() const;

    /// @brief Switch the contactor on/off.
    /// @param on The target state (true = on, false = off)
    /// @param wait_for_feedback Tell whether to wait and evaluate the feedback before returning.
    /// @return Returns true if the new state was reached successfully (based on feedback if configured),
    /// false otherwise.
    bool switch_state(bool on, bool wait_for_feedback);

    /// @brief Read the actual state (based on actuator line).
    bool get_state() const;

    /// @brief Read the actual state (based on feedback).
    types::cb_board_support::ContactorState get_feedback_state() const;

    /// @brief Check whether the actual feedback state matches the actuator state.
    /// @return Returns true if there both states differ, false otherwise.
    bool is_state_mismatch() const;

    /// @brief Tell the relay's feedback monitoring about an expected change which is not initiated by
    ///        ourself, e.g. because of a special serial wiring of multiple contactors.
    void set_expected_feedback_change(bool on);

    /// @brief Return the time to wait until the contactor can be closed again.
    std::chrono::milliseconds get_closing_delay_left() const;

    /// @brief Signal used to inform about unexpected state changes on the feedback contact.
    sigslot::signal<const std::string, types::cb_board_support::ContactorState> on_unexpected_change;

    /// @brief Feeds a string representation of the given CbTarragonContactorControlSimple
    ///        instance into an output stream.
    /// @return A reference to the output stream operated on.
    friend std::ostream& operator<<(std::ostream& os, const CbTarragonContactor& c);

private:
    /// @brief Stores the contactor description
    std::string name;

    /// @brief The feedback type used.
    types::cb_board_support::ContactorFeedbackType feedback_type;

    /// @brief The relay used to control the contactor and to evaluate the feedback (if used).
    std::unique_ptr<CbTarragonRelay> relay;
};
