// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH, Pionix GmbH and Contributors to EVerest

#include "ac_rcdImpl.hpp"

namespace module {
namespace ac_rcd {

void ac_rcdImpl::init() {
    this->mod->controller.on_rcm_state_change.connect([&](const enum rcm_state& new_rcm_state) {
        switch (new_rcm_state) {
        case rcm_state::RCM_STATE_NOT_CONFIGURED:
            // we only log this as debug for now to not pollute the log
            EVLOG_debug << "RCM is not configured in Safety Controller";
            break;

        case rcm_state::RCM_STATE_MONITORING:
            if (this->selftest_started) {
                EVLOG_info << "RCM self-test completed successfully";
                this->selftest_started = false;
            } else {
                EVLOG_info << "RCM monitoring is active";
            }

            if (this->selftest_error_reported.exchange(false)) {
                this->clear_error("ac_rcd/Selftest");
            }
            if (this->rcm_error_reported.exchange(false)) {
                this->clear_error("ac_rcd/DC");
            }
            break;

        case rcm_state::RCM_STATE_SELFTEST:
            if (this->selftest_started) {
                EVLOG_info << "RCM self-test is running";
            } else {
                EVLOG_info << "RCM test is running";
            }
            break;

        case rcm_state::RCM_STATE_ERROR:
            if (this->selftest_started) {
                EVLOG_error << "RCM self-test failed";
                this->selftest_started = false;

                if (!this->selftest_error_reported.exchange(true)) {
                    Everest::error::Error error_object = this->error_factory->create_error(
                        "ac_rcd/Selftest", "", "RCM self-test failed", Everest::error::Severity::High);
                    this->raise_error(error_object);
                }
            } else {
                EVLOG_error << "RCM reported error";

                if (!this->selftest_error_reported.exchange(true)) {
                    Everest::error::Error error_object = this->error_factory->create_error(
                        "ac_rcd/DC", "", "RCM tripped", Everest::error::Severity::High);
                    this->raise_error(error_object);
                }
            }
            break;
        default:
            break;
        }
    });
}

void ac_rcdImpl::ready() {
}

void ac_rcdImpl::handle_self_test() {
    try {
        EVLOG_info << "handle_self_test: forwarding request for RCM self-test to safety controller";
        this->selftest_started = true;
        this->mod->controller.start_rcm_selftest();
    } catch (std::exception& e) {
        EVLOG_error << e.what();
    }
}

bool ac_rcdImpl::handle_reset() {
    EVLOG_debug << "handle_reset: request for RCM reset ignored";
    return true;
}

} // namespace ac_rcd
} // namespace module
