// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "cb_chargesom_mcsImpl.hpp"

namespace module {
namespace mcs {

void cb_chargesom_mcsImpl::init() {

    // register our callback handlers

    this->mod->controller.on_id_change.connect([&](const enum cs2_id_state& new_id_state) {
        switch (new_id_state) {
        case cs2_id_state::CS2_ID_STATE_NOT_CONNECTED:
            EVLOG_info << "Plug removal detected, resetting HLC clearance";
            try {
                this->mod->controller.set_ccs_ready(false);
            } catch (std::exception& e) {
                EVLOG_error << e.what();
            }
            break;
        case cs2_id_state::CS2_ID_STATE_CONNECTED:
            EVLOG_info << "ID change detected: " << new_id_state;
            // FIXME we probably need to publish it somewhere
            break;
        default:
            EVLOG_info << "ID change detected: " << new_id_state;
        }
    });
}

void cb_chargesom_mcsImpl::ready() {
}

void cb_chargesom_mcsImpl::handle_mcs_hlc_enable(bool& value) {
    try {
        EVLOG_info << "handle_mcs_hlc_enable: " << std::boolalpha << value;
        this->mod->controller.set_ccs_ready(value);
    } catch (std::exception& e) {
        EVLOG_error << e.what();
    }
}

} // namespace mcs
} // namespace module
