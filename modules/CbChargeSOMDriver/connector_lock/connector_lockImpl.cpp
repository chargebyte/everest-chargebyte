// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH, Pionix GmbH and Contributors to EVerest

#include "connector_lockImpl.hpp"

namespace module {
namespace connector_lock {

void connector_lockImpl::init() {
    this->mod->controller.on_inlet_state_change.connect([&](const enum inlet_state& new_inlet_state) {
        switch (new_inlet_state) {
        case inlet_state::INLET_STATE_UNDEFINED:
            // do nothing
            break;

        case inlet_state::INLET_STATE_NOT_CONFIGURED:
            // do nothing
            break;

        case inlet_state::INLET_STATE_ERROR:
            if (!this->error_reported.exchange(true)) {
                Everest::error::Error error_object = this->error_factory->create_error(
                    "connector_lock/MREC1ConnectorLockFailure", "", "", Everest::error::Severity::High);
                this->raise_error(error_object);
            }
            break;

        default:
            // clear any previously raised error
            if (this->error_reported.exchange(false)) {
                this->clear_error("connector_lock/MREC1ConnectorLockFailure");
            }
            break;
        }
    });
}

void connector_lockImpl::ready() {
}

void connector_lockImpl::handle_lock() {
    try {
        EVLOG_info << "handle_lock: forwarding lock request to safety controller";
        this->mod->controller.inlet_lock();
    } catch (std::exception& e) {
        EVLOG_error << e.what();
    }
}

void connector_lockImpl::handle_unlock() {
    try {
        EVLOG_info << "handle_unlock: forwarding unlock request to safety controller";
        this->mod->controller.inlet_unlock();
    } catch (std::exception& e) {
        EVLOG_error << e.what();
    }
}

} // namespace connector_lock
} // namespace module
