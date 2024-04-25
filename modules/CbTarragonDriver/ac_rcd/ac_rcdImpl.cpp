// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include <chrono>
#include "ac_rcdImpl.hpp"

using namespace std::chrono_literals;

namespace module {
namespace ac_rcd {

void ac_rcdImpl::init() {
    if (this->mod->config.rcm_enable) {
        EVLOG_info << "Enabling RCM observation";

        this->rcm_controller = CbTarragonRCM(this->mod->config.rcm_fault_gpio_line_name,
                                             this->mod->config.rcm_fault_active_low);
        // start RCM observation thread
        this->rcm_observation_thread = std::thread(&ac_rcdImpl::rcm_observation_worker, this);
    }
}

ac_rcdImpl::~ac_rcdImpl() {
    this->termination_requested = true;
    // If thread is active wait until it is terminated
    if (this->rcm_observation_thread.joinable())
        this->rcm_observation_thread.join();
}

void ac_rcdImpl::ready() {
}

void ac_rcdImpl::handle_self_test() {
    // your code for cmd self_test goes here
}

bool ac_rcdImpl::handle_reset() {
    // your code for cmd reset goes here
    return true;
}

void ac_rcdImpl::rcm_observation_worker(void) {
    EVLOG_info << "RCM Observation Thread started";

    while(!this->termination_requested) {
        this->rcm_controller.wait_for_rcm_event(std::chrono::seconds(1));

        if(this->rcm_controller.is_rcm_tripped() && !this->rcm_tripped) {
            EVLOG_info << "RCM tripped";
            this->rcm_tripped = true;
            this->raise_ac_rcd_MREC2GroundFailure("RCM failure detected", Everest::error::Severity::High);
        }

        if (!this->rcm_controller.is_rcm_tripped() && this->rcm_tripped) {
            this->rcm_tripped = false;
            EVLOG_info << "RCM not tripped";

            this->request_clear_all_ac_rcd_MREC2GroundFailure();
        }
    }

    EVLOG_info << "RCM Observation Thread stopped";
}

} // namespace ac_rcd
} // namespace module
