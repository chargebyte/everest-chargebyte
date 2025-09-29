// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include <cerrno>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <system_error>
#include <thread>
// #include <libcbuart/logging.h>
#include "CbCpxDriver.hpp"
#include "configuration.h"

using namespace std::chrono_literals;

// // provide wrappers for libcbuart which add a tiny prefix and then passes on to EVerest logging
// static void libcbuart_log(bool debug, const char *format, va_list args) {
//     char msg[255];

//     vsnprintf(msg, sizeof(msg), format, args);

//     if (debug) {
//         EVLOG_debug << "libcbuart: " << msg;
//     } else {
//         EVLOG_error << "libcbuart: " << msg;
//     }
// }

// void libcbuart_debug_cb(const char *format, va_list args) {
//     libcbuart_log(true, format, args);
// }

// void libcbuart_error_cb(const char *format, va_list args) {
//     libcbuart_log(false, format, args);
// }

namespace module {

void CbCpxDriver::init() {
    types::evse_board_support::Connector_type connector_type =
        types::evse_board_support::string_to_connector_type(this->config.connector_type);
    bool is_pluggable = true;
    // bool is_pluggable = connector_type == types::evse_board_support::Connector_type::IEC62196Type2Socket;

    // register debug and error message callback functions
    // libcbuart_set_error_msg_cb(libcbuart_error_cb);
    // libcbuart_set_debug_msg_cb(libcbuart_debug_cb);

    // instantiate CPX controller
    this->controller = std::make_unique<CbCpx>(config);
    this->controller->init(is_pluggable);

    // initialize the interfaces now
    invoke_init(*p_evse_board_support);
    invoke_init(*p_temperatures);
    invoke_init(*p_ac_rcd);

    EVLOG_info << MODULE_DESCRIPTION << " (version: " << MODULE_VERSION << ")";
}

void CbCpxDriver::ready() {
    invoke_ready(*p_evse_board_support);
    invoke_ready(*p_temperatures);

    while (!this->termination_requested);
}

} // namespace module
