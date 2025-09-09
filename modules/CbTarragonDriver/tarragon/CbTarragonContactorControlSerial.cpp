// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include <chrono>
#include <iostream>
#include <string>
#include "CbTarragonContactor.hpp"
#include "CbTarragonContactorControl.hpp"
#include "CbTarragonContactorControlSerial.hpp"
#include <everest/logging.hpp>

CbTarragonContactorControlSerial::CbTarragonContactorControlSerial(
    std::unique_ptr<CbTarragonRelay> primary_relay, const std::string& primary_contactor_feedback_type,
    std::unique_ptr<CbTarragonRelay> secondary_relay, const std::string& secondary_contactor_feedback_type) :
    primary("Primary Contactor", std::move(primary_relay), primary_contactor_feedback_type),
    secondary("Secondary Contactor", std::move(secondary_relay), secondary_contactor_feedback_type) {

    EVLOG_info << this->primary.get_name() << " feedback type: '" << primary_contactor_feedback_type << "'";
    if (primary_contactor_feedback_type == "none")
        EVLOG_warning << "The " << this->primary.get_name()
                      << " has the feedback pin not connected. This is not recommended.";

    EVLOG_info << this->secondary.get_name() << " feedback type: '" << secondary_contactor_feedback_type << "'";
    if (secondary_contactor_feedback_type == "none")
        EVLOG_warning << "The " << this->secondary.get_name()
                      << " has the feedback pin not connected. This is not recommended.";

    this->primary.on_unexpected_change.connect(
        [&](const std::string& contactor_name, types::cb_board_support::ContactorState seen_contactor_state) {
            this->on_unexpected_change(contactor_name, seen_contactor_state);
        });

    this->secondary.on_unexpected_change.connect(
        [&](const std::string& contactor_name, types::cb_board_support::ContactorState seen_contactor_state) {
            this->on_unexpected_change(contactor_name, seen_contactor_state);
        });
}

bool CbTarragonContactorControlSerial::is_inconsistent_state(std::ostringstream& error_hint) const {
    if (this->primary.is_state_mismatch()) {
        error_hint << this->primary;
        return true;
    }

    if (this->secondary.is_state_mismatch()) {
        error_hint << this->secondary;
        return true;
    }

    return false;
}

bool CbTarragonContactorControlSerial::switch_state(bool on) {
    bool rv_primary, rv_secondary;

    // when closing, we have to consider the secondary first
    // Attention: we might see the feedback of the secondary first, when
    // the primary is switched!
    if (on) {
        if (this->phase_count == 3) {
            // don't wait here for the immediate feedback
            this->switch_contactor(this->secondary, on, false);
        }

        rv_primary = this->switch_contactor(this->primary, on);
        if (!rv_primary && this->phase_count == 3) {
            // switch back the secondary to be safe (but it should not be energized at all)
            this->switch_contactor(this->secondary, false, false);
            return false;
        }

        // now the secondary contactor should have also switched
        if (this->phase_count == 3) {
            auto secondary_contactor_state {this->secondary.get_feedback_state()};

            rv_secondary = secondary_contactor_state == types::cb_board_support::ContactorState::Closed;

            // switch_contactor would have raised an error in case we had waited
            // but since we didn't we have to take care here
            if (!rv_secondary) {
                auto actual_contactor_state {on ? types::cb_board_support::ContactorState::Open
                                                : types::cb_board_support::ContactorState::Closed};

                this->on_error(this->secondary.get_name(), on, actual_contactor_state);

                // switch back both to be on safe side
                this->switch_contactor(this->primary, false, false);
                this->switch_contactor(this->secondary, false, false);
            }
        } else {
            // not switched, so must be neutral for following and-condition in return
            rv_secondary = true;
        }

        return rv_primary and rv_secondary;
    }

    // when opening, we can always switch both contactors in the correct sequence;
    // here too, we might see the feedback of the secondary already when switching
    // the primary;
    // we also do not exit immediately in case of an error when switching the primary
    // but try at least to switch the secondary - depending on the actual wiring,
    // this could at least switch off phases 2 and 3 even if the primary contactor
    // was welded

    // tell the secondary about a possible switch - this can only happen in 3ph mode
    // otherwise the secondary would/should be open already
    if (this->phase_count == 3)
        this->secondary.set_expected_feedback_change(on);

    // Warning: Do not combine both calls via 'and' otherwise the compiler could
    // short-circuit the expression!
    rv_primary = this->switch_contactor(this->primary, on);
    rv_secondary = this->switch_contactor(this->secondary, on);

    return rv_primary and rv_secondary;
}

bool CbTarragonContactorControlSerial::get_state() const {
    // it is sufficient to check the primary contactor here
    return this->primary.get_state();
}

std::chrono::milliseconds CbTarragonContactorControlSerial::get_closing_delay_left() const {
    // since we always switch the primary contactor but sometimes the secondary not,
    // we can just look at the primary -> it covers the secondary completely
    return this->primary.get_closing_delay_left();
}

std::ostream& CbTarragonContactorControlSerial::dump(std::ostream& os) const {
    os << this->primary << ", " << this->secondary;
    return os;
}

std::ostream& operator<<(std::ostream& os, const CbTarragonContactorControlSerial& c) {
    return c.dump(os);
}
