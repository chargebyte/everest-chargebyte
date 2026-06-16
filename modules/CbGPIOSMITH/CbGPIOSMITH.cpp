// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH, Pionix GmbH and Contributors to EVerest
#include "CbGPIOSMITH.hpp"
#include <chargebyte/gpiodUtils.hpp>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

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

std::optional<pid_t> CbGPIOSMITH::find_process_pid_by_name(const std::string& process_name) {
    for (const auto& entry : std::filesystem::directory_iterator("/proc")) {
        if (!entry.is_directory()) {
            continue;
        }

        const std::string pid_str = entry.path().filename().string();
        if (pid_str.empty() ||
            !std::all_of(pid_str.begin(), pid_str.end(), [](unsigned char c) { return std::isdigit(c) != 0; })) {
            continue;
        }

        std::ifstream comm_file(entry.path() / "comm");
        if (!comm_file.is_open()) {
            continue;
        }

        std::string comm;
        std::getline(comm_file, comm);
        if (comm == process_name) {
            return static_cast<pid_t>(std::stoi(pid_str));
        }
    }

    return std::nullopt;
}

void CbGPIOSMITH::flush_journald_and_filesystems() {
    const auto journald_pid = find_process_pid_by_name("systemd-journal");
    if (journald_pid.has_value()) {
        if (::kill(*journald_pid, SIGUSR1) == 0) {
            EVLOG_debug << "Sent SIGUSR1 to systemd-journald (PID: " << *journald_pid << ")";
        } else {
            EVLOG_warning << "Failed to send SIGUSR1 to systemd-journald (PID: " << *journald_pid << ")";
        }
    } else {
        EVLOG_warning << "PID of systemd-journald not found";
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // flush filesystem buffers
    ::sync();
    EVLOG_debug << "Filesystem buffers flushed via sync()";
}

} // namespace module
