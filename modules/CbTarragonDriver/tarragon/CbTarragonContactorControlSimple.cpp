// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include <chrono>
#include <iostream>
#include <string>
#include "CbTarragonContactor.hpp"
#include "CbTarragonContactorControl.hpp"
#include "CbTarragonContactorControlSimple.hpp"
#include <everest/logging.hpp>

CbTarragonContactorControlSimple::CbTarragonContactorControlSimple(std::unique_ptr<CbTarragonRelay> relay,
                                                                   const std::string& contactor_feedback_type) :
    contactor("Contactor", std::move(relay), contactor_feedback_type) {

    EVLOG_info << this->contactor.get_name() << " feedback type: '" << contactor_feedback_type << "'";
    if (contactor_feedback_type == "none")
        EVLOG_warning << "The contactor has the feedback pin not connected. This is not recommended.";

    this->contactor.on_unexpected_change.connect(
        [&](const std::string& contactor_name, types::cb_board_support::ContactorState seen_contactor_state) {
            this->on_unexpected_change(contactor_name, seen_contactor_state);
        });
}

bool CbTarragonContactorControlSimple::is_inconsistent_state(std::ostringstream& error_hint) const {
    bool rv = this->contactor.is_state_mismatch();

    if (rv)
        error_hint << this->contactor;

    return rv;
}

bool CbTarragonContactorControlSimple::switch_state(bool on) {
    return this->switch_contactor(this->contactor, on);
}

bool CbTarragonContactorControlSimple::get_state() const {
    return this->contactor.get_state();
}

std::chrono::milliseconds CbTarragonContactorControlSimple::get_closing_delay_left() const {
    return this->contactor.get_closing_delay_left();
}

std::ostream& CbTarragonContactorControlSimple::dump(std::ostream& os) const {
    os << this->contactor;
    return os;
}

std::ostream& operator<<(std::ostream& os, const CbTarragonContactorControlSimple& c) {
    return c.dump(os);
}
