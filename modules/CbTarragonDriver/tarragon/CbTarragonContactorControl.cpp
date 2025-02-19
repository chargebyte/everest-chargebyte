// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include <chrono>
#include <string>
#include <sigslot/signal.hpp>
#include "CbTarragonContactor.hpp"
#include "CbTarragonContactorControl.hpp"

using namespace std::chrono_literals;

bool CbTarragonContactorControl::switch_contactor(CbTarragonContactor& contactor, bool on, bool wait_for_feedback) {
    bool rv;

    // reject power on in case emergency flag is set
    if (this->is_emergency && on)
        return false;

    rv = contactor.switch_state(on, wait_for_feedback);

    if (!rv) {
        // Note: we failed to switch, that means we've 'seen' still the old state
        types::cb_board_support::ContactorState actual_contactor_state {
            on ? types::cb_board_support::ContactorState::Open : types::cb_board_support::ContactorState::Closed};

        this->on_error(contactor.get_name(), on, actual_contactor_state);
    }

    return rv;
}

bool CbTarragonContactorControl::open() {
    return this->switch_state(false);
}

void CbTarragonContactorControl::switch_phase_count(bool use_3phases) {
    this->phase_count = use_3phases ? 3 : 1;
}

std::ostream& CbTarragonContactorControl::dump(std::ostream& os) const {
    os << "CbTarragonContactorControl";
    return os;
}

std::ostream& operator<<(std::ostream& os, const CbTarragonContactorControl& c) {
    return c.dump(os);
}
