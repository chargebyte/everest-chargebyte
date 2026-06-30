// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH, Pionix GmbH and Contributors to EVerest
#include "CbGPIOSMITH.hpp"
#include <everest/system/safe_system.hpp>
#include <chargebyte/gpiodUtils.hpp>
#include <string>

namespace module {

void CbGPIOSMITH::init() {
    gpiod::line_settings line_settings;
    line_settings.set_direction(gpiod::line::direction::OUTPUT);
    line_settings.set_active_low(this->config.gpio_active_low);
    line_settings.set_output_value(gpiod::line::value::INACTIVE);

    this->gpio = std::make_unique<gpiod::line_request>(
        get_gpioline_by_name(this->config.gpio_line_name, "CbGPIOSMITH", line_settings));

    Everest::error::ErrorCallback error_callback = [this](const Everest::error::Error& error) {
        if (error.type == "evse_board_support/MREC17EVSEContactorFault") {
            if (this->charging_started_seen) {
                EVLOG_info << "Contactor Error Detected - Triggering Fallback Safety Circuit";

                flush_journald_and_filesystems();

                this->gpio->set_value(this->gpio->offsets()[0], gpiod::line::value::ACTIVE);
            } else {
                EVLOG_info << "Bootup Contactor Error Detected - Not Triggering Fallback Safety Circuit";
            }
        }
    };

    Everest::error::ErrorCallback error_cleared_callback = [this](const Everest::error::Error& error) {
        (void)error;
        // not important for our use-case here
    };

    subscribe_global_all_errors(error_callback, error_cleared_callback);

    // register for all session events to see state transitions
    // (otherwise we would also react on bootup contactor errors and this would lead to
    // a reboot loop when somebody wants to power up and investigate
    for (const auto& m : this->r_evse_manager) {
        m->subscribe_session_event([this](types::evse_manager::SessionEvent e) {
            if (e.event == types::evse_manager::SessionEventEnum::ChargingStarted) {
                this->charging_started_seen = true;
            }
        });
    }
}

void CbGPIOSMITH::ready() {
}

void CbGPIOSMITH::flush_journald_and_filesystems() {
    EVLOG_debug << "Calling `journalctl --sync`";

    const auto result = everest::lib::system::safe_system("/bin/journalctl", {"journalctl", "--sync"});
    if (result.status == everest::lib::system::CMD_SUCCESS && result.code == 0 && !result.timeout) {
        EVLOG_debug << "Successfully synchronized journal.";
    } else {
        EVLOG_warning << "Failed to synchronize journal: status="
                      << everest::lib::system::cmd_execution_status_to_string(result.status) << ", code=" << result.code
                      << ", timeout=" << (result.timeout ? "true" : "false");
    }

    // flush filesystem buffers
    ::sync();
    EVLOG_debug << "Filesystem buffers flushed via sync()";
}

} // namespace module
