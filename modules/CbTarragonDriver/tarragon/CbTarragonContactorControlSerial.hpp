// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once
#include <chrono>
#include <memory>
#include <iostream>
#include <sstream>
#include <string>
#include "CbTarragonContactor.hpp"
#include "CbTarragonContactorControl.hpp"
#include "CbTarragonRelay.hpp"

///
/// This class implements a 'serial' wiring contactor setup, i.e. with support for phase count switching.
/// This means, that the primary contactor either switches all 3 phases or at least the first one,
/// and the secondary contactor switches phase 2 and 3. But the important aspect here is that the
/// secondary contactor is first switched on, and switched off last, while the primary contactor is
/// switched on last, and switched off first. The sequence is required to present a consistent
/// "these phases are available" view to the car - in other words, the phases do not arrive with
/// a time lag at the car.
///
class CbTarragonContactorControlSerial : public CbTarragonContactorControl {

public:
    /// @brief Constructor.
    /// @param primary_relay A pointer to an instance of CbTarragonRelay.
    /// @param primary_contactor_feedback_type Defines the logic behind the feedback of the primary contactor (no =
    /// normally open, nc = normally close, none = no feedback).
    /// @param secondary_relay A pointer to an instance of CbTarragonRelay.
    /// @param secondary_contactor_feedback_type Defines the logic behind the feedback of the secondary contactor (no =
    /// normally open, nc = normally close, none = no feedback).
    CbTarragonContactorControlSerial(std::unique_ptr<CbTarragonRelay> primary_relay,
                                     const std::string& primary_contactor_feedback_type,
                                     std::unique_ptr<CbTarragonRelay> secondary_relay,
                                     const std::string& secondary_contactor_feedback_type);

    virtual bool is_inconsistent_state(std::ostringstream& error_hint) const override;
    virtual bool switch_state(bool on) override;
    virtual bool get_state() const override;
    virtual std::chrono::milliseconds get_closing_delay_left() const override;

    /// @brief Feeds a string representation of the given CbTarragonContactorControlSerial
    ///        instance into an output stream.
    /// @return A reference to the output stream operated on.
    friend std::ostream& operator<<(std::ostream& os, const CbTarragonContactorControlSerial& c);

protected:
    /// @brief The primary contactor and its feedback abstraction.
    CbTarragonContactor primary;

    /// @brief The secondary contactor and its feedback abstraction.
    CbTarragonContactor secondary;

    /// @brief Split `switch_state` into two dedicated functions so that we can override in derived classes.
    virtual bool switch_state_on();

    /// @brief Split `switch_state` into two dedicated functions so that we can override in derived classes.
    virtual bool switch_state_off();

    /// @brief Helper to feed a string representation into an output stream.
    /// @param os Output stream reference to operate on.
    /// @return A reference to the output stream operated on.
    virtual std::ostream& dump(std::ostream& os) const override;
};
