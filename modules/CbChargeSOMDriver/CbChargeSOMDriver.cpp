// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include <cstdarg>
#include <system_error>
#include <ra-utils/logging.h>
#include "CbChargeSOMDriver.hpp"
#include "configuration.h"

using namespace std::chrono_literals;

static bool enable_ra_utils_debug_msg;

// provide wrappers for libra-utils which add a tiny prefix and then passes on to EVerest logging
static void ra_utils_log(bool debug, const char* format, va_list args) {
    char msg[255];

    vsnprintf(msg, sizeof(msg), format, args);

    if (debug) {
        EVLOG_debug << msg;
    } else {
        EVLOG_error << msg;
    }
}

void ra_utils_debug_cb(const char* format, va_list args) {
    if (enable_ra_utils_debug_msg) {
        ra_utils_log(true, format, args);
    }
}

void ra_utils_error_cb(const char* format, va_list args) {
    ra_utils_log(false, format, args);
}

namespace module {

void CbChargeSOMDriver::init() {
    types::evse_board_support::Connector_type connector_type =
        types::evse_board_support::string_to_connector_type(this->config.connector_type);
    bool is_pluggable = connector_type == types::evse_board_support::Connector_type::IEC62196Type2Socket;

    // register debug and error message callback functions
    ra_utils_set_debug_msg_cb(ra_utils_debug_cb);
    ra_utils_set_error_msg_cb(ra_utils_error_cb);
    enable_ra_utils_debug_msg = this->config.serial_debug;

    EVLOG_info << MODULE_DESCRIPTION << " (version: " << MODULE_VERSION << ")";

    // instantiate UART controller object for communication with safety controller
    this->controller.init(this->config.reset_gpio_line_name, this->config.reset_active_low, this->config.serial_port,
                          is_pluggable, this->config.serial_trace);

    EVLOG_info << "Safety Controller Firmware: " << this->controller.get_fw_info();

    // initialize the interfaces now
    invoke_init(*p_evse_board_support);
    invoke_init(*p_temperatures);
}

void CbChargeSOMDriver::ready() {
    invoke_ready(*p_evse_board_support);
    invoke_ready(*p_temperatures);
}

} // namespace module
