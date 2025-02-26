// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include <chrono>
#include <iostream>
#include <string>
#include "CbTarragonContactor.hpp"
#include "CbTarragonContactorControl.hpp"
#include "CbTarragonContactorControlMutual.hpp"
#include <everest/logging.hpp>

CbTarragonContactorControlMutual::CbTarragonContactorControlMutual(std::unique_ptr<CbTarragonRelay> relay_3ph,
                                                                   const std::string& contactor_3ph_feedback_type,
                                                                   std::unique_ptr<CbTarragonRelay> relay_1ph,
                                                                   const std::string& contactor_1ph_feedback_type) :
    contactor_3ph("3ph Contactor", std::move(relay_3ph), contactor_3ph_feedback_type),
    contactor_1ph("1ph Contactor", std::move(relay_1ph), contactor_1ph_feedback_type) {

    EVLOG_info << this->contactor_3ph.get_name() << " feedback type: '" << contactor_3ph_feedback_type << "'";
    if (contactor_3ph_feedback_type == "none")
        EVLOG_warning << "The " << this->contactor_3ph.get_name()
                      << " has the feedback pin not connected. This is not recommended.";

    EVLOG_info << this->contactor_1ph.get_name() << " feedback type: '" << contactor_1ph_feedback_type << "'";
    if (contactor_1ph_feedback_type == "none")
        EVLOG_warning << "The " << this->contactor_1ph.get_name()
                      << " has the feedback pin not connected. This is not recommended.";

    this->contactor_3ph.on_unexpected_change.connect(
        [&](const std::string& contactor_name, types::cb_board_support::ContactorState seen_contactor_state) {
            this->on_unexpected_change(contactor_name, seen_contactor_state);
        });

    this->contactor_1ph.on_unexpected_change.connect(
        [&](const std::string& contactor_name, types::cb_board_support::ContactorState seen_contactor_state) {
            this->on_unexpected_change(contactor_name, seen_contactor_state);
        });
}

bool CbTarragonContactorControlMutual::is_inconsistent_state(std::ostringstream& error_hint) const {
    if (this->contactor_3ph.is_state_mismatch()) {
        error_hint << this->contactor_3ph;
        return true;
    }

    if (this->contactor_1ph.is_state_mismatch()) {
        error_hint << this->contactor_1ph;
        return true;
    }

    return false;
}

bool CbTarragonContactorControlMutual::switch_state(bool on) {
    if (this->phase_count == 3)
        return this->switch_contactor(this->contactor_3ph, on);
    else
        return this->switch_contactor(this->contactor_1ph, on);
}

bool CbTarragonContactorControlMutual::get_state() const {
    // we can combine both states by OR-ing
    return this->contactor_3ph.get_state() or this->contactor_1ph.get_state();
}

std::chrono::milliseconds CbTarragonContactorControlMutual::get_closing_delay_left() const {
    if (this->phase_count == 3)
        return this->contactor_3ph.get_closing_delay_left();
    else
        return this->contactor_1ph.get_closing_delay_left();
}

std::ostream& CbTarragonContactorControlMutual::dump(std::ostream& os) const {
    os << this->contactor_3ph << ", " << this->contactor_1ph;
    return os;
}

std::ostream& operator<<(std::ostream& os, const CbTarragonContactorControlMutual& c) {
    return c.dump(os);
}
