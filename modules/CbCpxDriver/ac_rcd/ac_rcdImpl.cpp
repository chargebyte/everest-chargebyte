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
        // EVLOG_info << "RCM GPIO: " << this->mod->config.rcm_fault_gpio_line_name;
        // EVLOG_info << "RCM GPIO polarity: " << (this->mod->config.rcm_fault_active_low ? "active_low" : "active_high");

        // this->mod->rcm_controller =
        //     CbTarragonRCM(this->mod->config.rcm_fault_gpio_line_name, this->mod->config.rcm_fault_active_low);
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

    while (!this->termination_requested) {
        // this->mod->rcm_controller.wait_for_rcm_event(1s);

        if (0==1) {
        // if (this->mod->rcm_controller.is_rcm_tripped() && !this->rcm_tripped) {
            // signal emergency state to evse_board_support interface for open the contactor immediately
            // this->mod->rcm_controller.on_change(true);

            EVLOG_debug << "RCM tripped";
            this->rcm_tripped = true;

            Everest::error::Error error_object = this->error_factory->create_error(
                "ac_rcd/MREC2GroundFailure", "", "RCM failure detected", Everest::error::Severity::High);
            this->raise_error(error_object);
        }

        if (0==1) {
        // if (!this->mod->rcm_controller.is_rcm_tripped() && this->rcm_tripped) {
            EVLOG_debug << "RCM not tripped";
            this->rcm_tripped = false;

            // signal released emergency state to evse_board_support interface
            // this->mod->rcm_controller.on_change(false);

            this->clear_error("ac_rcd/MREC2GroundFailure");
        }
    }

    EVLOG_info << "RCM Observation Thread stopped";
}

} // namespace ac_rcd
} // namespace module
