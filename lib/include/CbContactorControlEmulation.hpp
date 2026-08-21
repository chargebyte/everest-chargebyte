// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once
#include <chrono>
#include <iostream>
#include <sstream>
#include <string>
#include "CbContactor.hpp"
#include "CbContactorControl.hpp"

///
/// This class implements an emulated contactor, eg. a single contactor
/// without phase count switching.
///
class CbContactorControlEmulation : public CbContactorControl {

public:
    /// @brief Constructor.
    CbContactorControlEmulation() = default;

    /// @brief Destructor.
    virtual ~CbContactorControlEmulation() = default;

    virtual bool is_inconsistent_state(std::ostringstream& error_hint) const override;
    virtual bool switch_state(bool on) override;
    virtual bool get_state() const override;
    virtual std::chrono::milliseconds get_closing_delay_left() const override;

    /// @brief Feeds a string representation of the given CbContactorControlEmulation
    ///        instance into an output stream.
    /// @return A reference to the output stream operated on.
    friend std::ostream& operator<<(std::ostream& os, const CbContactorControlEmulation& c);

private:
    /// @brief Remember latest switching state
    bool is_on {false};

    /// @brief Helper to feed a string representation into an output stream.
    /// @param os Output stream reference to operate on.
    /// @return A reference to the output stream operated on.
    virtual std::ostream& dump(std::ostream& os) const override;
};
