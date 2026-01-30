// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include "CbCPXDriver.hpp"
#include "configuration.h"

using namespace std::chrono_literals;

namespace module {

void CbCPXDriver::init() {
    types::evse_board_support::Connector_type connector_type =
        types::evse_board_support::string_to_connector_type(this->config.connector_type);
    const bool is_pluggable = connector_type == types::evse_board_support::Connector_type::IEC62196Type2Socket;

    // instantiate CPX controller
    this->controller = std::make_unique<CbCPX>(this->config.device_id, this->config.CAN_interface, this->config.CAN_bitrate);
    this->controller->init(is_pluggable);

    // initialize the interfaces now
    invoke_init(*this->p_evse_board_support);
    invoke_init(*this->p_temperatures);

    EVLOG_info << MODULE_DESCRIPTION << " (version: " << MODULE_VERSION << ")";
}

void CbCPXDriver::ready() {
    invoke_ready(*this->p_evse_board_support);
    invoke_ready(*this->p_temperatures);
}

} // namespace module
