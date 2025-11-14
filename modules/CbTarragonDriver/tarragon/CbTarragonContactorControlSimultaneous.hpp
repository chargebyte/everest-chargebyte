// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once
#include <memory>
#include <iostream>
#include <sstream>
#include <string>
#include "CbTarragonContactor.hpp"
#include "CbTarragonContactorControl.hpp"
#include "CbTarragonContactorControlSerial.hpp"
#include "CbTarragonRelay.hpp"

///
/// This class implements a 'simultaneous' wiring contactor setup for phase count switching.
/// In this setup, a primary contactor switches neutral and one phase, a secondary contactor
/// switches both remaining ones. The difference to 'serial' setup is, that in this class
/// it is guaranteed that both contactors are switched at the same time using libgpiod and
/// the Linux's kernel feature to switch two GPIOs within the very same GPIO bank.
/// This is enforced already by passing different `CbActuator` references during the relay
/// instantiations. In this class there are only smaller differences thus we can derive it
/// from `CbTarragonContactorControlSerial`.
///
class CbTarragonContactorControlSimultaneous : public CbTarragonContactorControlSerial {

public:
    /// @brief Constructor.
    /// @param primary_relay A pointer to an instance of CbTarragonRelay.
    /// @param primary_contactor_feedback_type Defines the logic behind the feedback of the primary contactor (no =
    /// normally open, nc = normally close, none = no feedback).
    /// @param secondary_relay A pointer to an instance of CbTarragonRelay.
    /// @param secondary_contactor_feedback_type Defines the logic behind the feedback of the secondary contactor (no =
    /// normally open, nc = normally close, none = no feedback).
    CbTarragonContactorControlSimultaneous(std::unique_ptr<CbTarragonRelay> primary_relay,
                                           const std::string& primary_contactor_feedback_type,
                                           std::unique_ptr<CbTarragonRelay> secondary_relay,
                                           const std::string& secondary_contactor_feedback_type);

    /// @brief Destructor.
    virtual ~CbTarragonContactorControlSimultaneous() = default;

    virtual bool get_state() const override;

    /// @brief Feeds a string representation of the given CbTarragonContactorControlSimultaneous
    ///        instance into an output stream.
    /// @return A reference to the output stream operated on.
    friend std::ostream& operator<<(std::ostream& os, const CbTarragonContactorControlSimultaneous& c);

protected:
    /// @brief Compared to base class, the order is little bit different.
    virtual bool switch_state_off() override;
};
