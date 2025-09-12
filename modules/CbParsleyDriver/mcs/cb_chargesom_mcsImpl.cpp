// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "cb_chargesom_mcsImpl.hpp"

namespace module {
namespace mcs {

void cb_chargesom_mcsImpl::init() {
    // register our callback handlers
    this->mod->controller.on_id_change.connect([&](const types::cb_board_support::IDState& id_state) {
        EVLOG_info << "ID change detected: " << this->last_id_state << " → " << id_state;
        this->publish_id_state(id_state);
        this->last_id_state = id_state;
    });

    this->mod->controller.on_ce_change.connect([&](const types::cb_board_support::CEState& ce_state) {
        EVLOG_debug << "publishing new CE state: " << ce_state;
        this->publish_ce_state(ce_state);
    });
}

void cb_chargesom_mcsImpl::ready() {
}

bool cb_chargesom_mcsImpl::handle_mcs_hlc_enable(bool& value) {
    try {
        EVLOG_info << "handle_mcs_hlc_enable: " << std::boolalpha << value;
        this->mod->controller.set_ccs_ready(value);
        return true;
    } catch (std::exception& e) {
        EVLOG_error << e.what();
    }
    return false;
}

} // namespace mcs
} // namespace module
