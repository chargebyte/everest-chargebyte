// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include <chrono>
#include "ac_rcdImpl.hpp"
#include "evse_board_support/evse_board_supportImpl.hpp"

using namespace std::chrono_literals;

namespace module {
namespace ac_rcd {

void ac_rcdImpl::init() {
    if (this->mod->config.rcm_enable) {
        EVLOG_info << "Enabling RCM observation";
        EVLOG_info << "RCM GPIO: " << this->mod->config.rcm_fault_gpio_line_name;
        EVLOG_info << "RCM GPIO polarity: " << (this->mod->config.rcm_fault_active_low ? "active_low" : "active_high");

        this->rcm_controller = CbTarragonRCM(this->mod->config.rcm_fault_gpio_line_name,
                                             this->mod->config.rcm_fault_active_low);
    }
}

ac_rcdImpl::~ac_rcdImpl() {
    this->termination_requested = true;
    // If thread is active wait until it is terminated
    if (this->rcm_observation_thread.joinable())
        this->rcm_observation_thread.join();
}

void ac_rcdImpl::ready() {
    if (this->mod->config.rcm_enable) {
        // start RCM observation thread
        this->rcm_observation_thread = std::thread(&ac_rcdImpl::rcm_observation_worker, this);
    }
}

void ac_rcdImpl::handle_self_test() {
	// TODO The self test needs to be implemented later ...
}

bool ac_rcdImpl::handle_reset() {
	// Currently we assume that the RCD reset itself
    return true;
}

void ac_rcdImpl::rcm_observation_worker(void) {
    EVLOG_info << "RCM Observation Thread started";

    // workaround for already tripped RCM at start
    std::this_thread::sleep_for(1s);

    while(!this->termination_requested) {
        this->rcm_controller.wait_for_rcm_event(1s);

        if(this->rcm_controller.is_rcm_tripped() && !this->rcm_tripped) {
            // signal emergency state to evse_board_support interface for open the contactor immediately
            module::evse_board_support::evse_board_supportImpl::set_emergency_state(true);
            this->rcm_tripped = true;
            this->raise_ac_rcd_MREC2GroundFailure("RCM failure detected", Everest::error::Severity::High);
            EVLOG_info << "RCM tripped";
        }

        if (!this->rcm_controller.is_rcm_tripped() && this->rcm_tripped) {
            this->rcm_tripped = false;
            EVLOG_info << "RCM not tripped";
            // signal released emergency state to evse_board_support interface
            module::evse_board_support::evse_board_supportImpl::set_emergency_state(false);
            this->request_clear_all_ac_rcd_MREC2GroundFailure();
        }
    }

    EVLOG_info << "RCM Observation Thread stopped";
}

} // namespace ac_rcd
} // namespace module
