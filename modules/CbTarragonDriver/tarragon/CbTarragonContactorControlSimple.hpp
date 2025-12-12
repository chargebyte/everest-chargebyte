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
/// This class implements a standard/usual contactor setup, eg. a single contactor
/// without phase count switching.
///
class CbTarragonContactorControlSimple : public CbTarragonContactorControl {

public:
    /// @brief Constructor.
    /// @param relay A pointer to an instance of CbTarragonRelay.
    /// @param contactor_feedback_type Defines the logic behind the feedback (no = normally open, nc = normally close,
    /// none = no feedback).
    CbTarragonContactorControlSimple(std::unique_ptr<CbTarragonRelay> relay,
                                     const std::string& contactor_feedback_type);

    /// @brief Destructor.
    virtual ~CbTarragonContactorControlSimple() = default;

    virtual bool is_inconsistent_state(std::ostringstream& error_hint) const override;
    virtual bool switch_state(bool on) override;
    virtual bool get_state() const override;
    virtual std::chrono::milliseconds get_closing_delay_left() const override;

    /// @brief Feeds a string representation of the given CbTarragonContactorControlSimple
    ///        instance into an output stream.
    /// @return A reference to the output stream operated on.
    friend std::ostream& operator<<(std::ostream& os, const CbTarragonContactorControlSimple& c);

private:
    /// @brief The contactor and it's feedback abstraction.
    CbTarragonContactor contactor;

    /// @brief Helper to feed a string representation into an output stream.
    /// @param os Output stream reference to operate on.
    /// @return A reference to the output stream operated on.
    virtual std::ostream& dump(std::ostream& os) const override;
};
