// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include <cerrno>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <system_error>
#include <thread>
#include <libcbuart/logging.h>
#include "CbChargeSOMDriver.hpp"
#include "configuration.h"

using namespace std::chrono_literals;

// provide wrappers for libcbuart which add a tiny prefix and then passes on to EVerest logging
static void libcbuart_log(bool debug, const char *format, va_list args) {
    char msg[255];

    vsnprintf(msg, sizeof(msg), format, args);

    if (debug) {
        EVLOG_debug << "libcbuart: " << msg;
    } else {
        EVLOG_error << "libcbuart: " << msg;
    }
}

void libcbuart_debug_cb(const char *format, va_list args) {
    libcbuart_log(true, format, args);
}

void libcbuart_error_cb(const char *format, va_list args) {
    libcbuart_log(false, format, args);
}

namespace module {

void CbChargeSOMDriver::init() {
    types::evse_board_support::Connector_type connector_type =
        types::evse_board_support::string_to_connector_type(this->config.connector_type);
    bool is_pluggable = connector_type == types::evse_board_support::Connector_type::IEC62196Type2Socket;

    // register debug and error message callback functions
    libcbuart_set_error_msg_cb(libcbuart_error_cb);
    libcbuart_set_debug_msg_cb(libcbuart_debug_cb);

    // instantiate UART controller object for communication with safety controller
    this->controller.init(this->config.serial_port, is_pluggable);

    // initialize the interfaces now
    invoke_init(*p_evse_board_support);

    EVLOG_info << MODULE_DESCRIPTION << " (version: " << MODULE_VERSION << ")";
}

void CbChargeSOMDriver::ready() {
    invoke_ready(*p_evse_board_support);
    int etimedout_log_threshold = 2;
    int etimedout_count = 0;

    while (!this->termination_requested) {
        try {
            this->controller.sync();

            // reset the counter
            etimedout_count = 0;
        } catch (std::system_error& e) {
            // count timeouts
            if (e.code().value() == ETIMEDOUT)
                etimedout_count++;

            // and log only timeouts above our threshold value, or
            // errors which are not timeouts
            if (etimedout_count >= etimedout_log_threshold ||
                e.code().value() != ETIMEDOUT) {
                EVLOG_error << e.what();
            }

            // give the safety controller the chance to recover
            std::this_thread::sleep_for(this->controller.get_recovery_delay_ms());
        }

        // FIXME this should be removed since we want to query the safety controller
        //       as fast as possible and the usage of the recovery delay is confusing
        std::this_thread::sleep_for(this->controller.get_recovery_delay_ms());
    }
}

} // namespace module
