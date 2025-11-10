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
/// This class implements a 'mutual' wiring contactor setup, i.e. with support for phase count switching.
/// This means, that the first contactor switches all 3 phases and the second contactor switches
/// only a single phase. Both contactors are used mutually exclusive.
///
class CbTarragonContactorControlMutual : public CbTarragonContactorControl {

public:
    /// @brief Constructor.
    /// @param relay_3ph A pointer to an instance of CbTarragonRelay.
    /// @param contactor_3ph_feedback_type Defines the logic behind the feedback of the 3ph contactor (no = normally
    /// open, nc = normally close, none = no feedback).
    /// @param relay_1ph A pointer to an instance of CbTarragonRelay.
    /// @param contactor_1ph_feedback_type Defines the logic behind the feedback of the 1ph contactor (no = normally
    /// open, nc = normally close, none = no feedback).
    CbTarragonContactorControlMutual(std::unique_ptr<CbTarragonRelay> relay_3ph,
                                     const std::string& contactor_3ph_feedback_type,
                                     std::unique_ptr<CbTarragonRelay> relay_1ph,
                                     const std::string& contactor_1ph_feedback_type);

    /// @brief Destructor.
    virtual ~CbTarragonContactorControlMutual() = default;

    virtual bool is_inconsistent_state(std::ostringstream& error_hint) const override;
    virtual bool switch_state(bool on) override;
    virtual bool get_state() const override;
    virtual std::chrono::milliseconds get_closing_delay_left() const override;

    /// @brief Feeds a string representation of the given CbTarragonContactorControlMutual
    ///        instance into an output stream.
    /// @return A reference to the output stream operated on.
    friend std::ostream& operator<<(std::ostream& os, const CbTarragonContactorControlMutual& c);

private:
    /// @brief The contactor which switches all 3 phases and its feedback abstraction.
    CbTarragonContactor contactor_3ph;

    /// @brief The contactor which switches only 1 phase and its feedback abstraction.
    CbTarragonContactor contactor_1ph;

    /// @brief Helper to feed a string representation into an output stream.
    /// @param os Output stream reference to operate on.
    /// @return A reference to the output stream operated on.
    virtual std::ostream& dump(std::ostream& os) const override;
};
