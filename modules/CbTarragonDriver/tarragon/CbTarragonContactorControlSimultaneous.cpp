// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include <chrono>
#include <iostream>
#include <string>
#include "CbTarragonContactor.hpp"
#include "CbTarragonContactorControl.hpp"
#include "CbTarragonContactorControlSerial.hpp"
#include "CbTarragonContactorControlSimultaneous.hpp"

CbTarragonContactorControlSimultaneous::CbTarragonContactorControlSimultaneous(
    std::unique_ptr<CbTarragonRelay> primary_relay, const std::string& primary_contactor_feedback_type,
    std::unique_ptr<CbTarragonRelay> secondary_relay, const std::string& secondary_contactor_feedback_type) :
    CbTarragonContactorControlSerial(std::move(primary_relay), primary_contactor_feedback_type,
                                     std::move(secondary_relay), secondary_contactor_feedback_type) {
}

bool CbTarragonContactorControlSimultaneous::get_state() const {
    // we must combine both states by OR-ing
    return this->primary.get_state() or this->secondary.get_state();
}

bool CbTarragonContactorControlSimultaneous::switch_state_off() {
    bool rv_primary, rv_secondary;

    // when opening, we here have to open the secondary first since the call
    // is cached until the primary contactor request is executed;
    // thus we will see the feedback of the secondary first when switching
    // the primary, so we can first check after the primary switched

    // switch the secondary off first
    this->switch_contactor(this->secondary, false, false);

    // now switch the primary off which should also switch the secondary at the same time
    rv_primary = this->switch_contactor(this->primary, false);

    // now check state of secondary
    rv_secondary = this->secondary.wait_for_feedback();

    // switch_contactor would have raised an error in case we had waited
    // but since we didn't we have to take care here
    if (!rv_secondary)
        this->on_error(this->secondary.get_name(), false, types::cb_board_support::ContactorState::Closed);

    return rv_primary and rv_secondary;
}

std::ostream& operator<<(std::ostream& os, const CbTarragonContactorControlSimultaneous& c) {
    return c.dump(os);
}
