// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include <cerrno>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <system_error>
#include <thread>
#include "CbCpxDriver.hpp"
#include "configuration.h"

using namespace std::chrono_literals;

namespace module {

void CbCpxDriver::init() {
    types::evse_board_support::Connector_type connector_type =
        types::evse_board_support::string_to_connector_type(this->config.connector_type);
    bool is_pluggable = connector_type == types::evse_board_support::Connector_type::IEC62196Type2Socket;

    // instantiate CPX controller
    this->controller = std::make_unique<CbCpx>(config);
    this->controller->init(is_pluggable);

    // initialize the interfaces now
    invoke_init(*p_evse_board_support);
    invoke_init(*p_temperatures);

    EVLOG_info << MODULE_DESCRIPTION << " (version: " << MODULE_VERSION << ")";
}

void CbCpxDriver::ready() {
    invoke_ready(*p_evse_board_support);
    invoke_ready(*p_temperatures);

    while (!this->termination_requested) {
        // wait for 1 second to keep CPU load low
        std::this_thread::sleep_for(std::chrono::seconds(1));
    };
}

} // namespace module
