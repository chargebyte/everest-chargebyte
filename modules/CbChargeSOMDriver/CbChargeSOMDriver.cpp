// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include <cstdarg>
#include <system_error>
#include <ra-utils/logging.h>
#include "CbChargeSOMDriver.hpp"
#include "CbCarrierBoardRelay.hpp"
#include <CbContactorControlEmulation.hpp>
#include <CbContactorControlSimple.hpp>
#include <CbContactorControlSerial.hpp>
#include <CbContactorControlSimultaneous.hpp>
#include <CbContactorControlMutual.hpp>
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
    types::evse_board_support::Connector_type connector_type;

    // the interface and enum does not support proper types yet, so for DC we
    // fake for the moment a fixed cable
    if (this->config.connector_type == "cCCS2") {
        connector_type = types::evse_board_support::Connector_type::IEC62196Type2Cable;
    } else {
        connector_type = types::evse_board_support::string_to_connector_type(this->config.connector_type);
    }

    bool is_pluggable = connector_type == types::evse_board_support::Connector_type::IEC62196Type2Socket;

    // register debug and error message callback functions
    ra_utils_set_debug_msg_cb(ra_utils_debug_cb);
    ra_utils_set_error_msg_cb(ra_utils_error_cb);
    enable_ra_utils_debug_msg = this->config.serial_debug;

    EVLOG_info << MODULE_DESCRIPTION << " (version: " << MODULE_VERSION << ")";

    // instantiate UART controller object for communication with safety controller
    this->controller.init(this->config.reset_gpio_line_name, this->config.reset_active_low, this->config.serial_port,
                          is_pluggable, this->config.serial_trace, this->config.can_mirror_device);

    EVLOG_info << "Safety Controller Firmware: " << this->controller.get_fw_info();

    // determine AC vs DC mode and corresponding contactor configuration based on configured connector type
    if (this->config.connector_type == "cCCS2") {
        // DC mode

        if (this->config.dc_contactor_wiring == "none") {
            this->contactor_controller = std::make_unique<CbContactorControlEmulation>();

        } else if (this->config.dc_contactor_wiring == "single") {
            auto relay = std::make_unique<CbCarrierBoardRelay>(
                this->controller,
                std::initializer_list<CbCarrierBoardRelay::Contactor> {CbCarrierBoardRelay::Contactor::Contactor1},
                "Contactor 1");

            this->contactor_controller =
                std::make_unique<CbContactorControlSimple>(std::move(relay), this->config.contactor_1_feedback_type);

        } else if (this->config.dc_contactor_wiring == "dual") {
            auto primary = std::make_unique<CbCarrierBoardRelay>(
                this->controller,
                std::initializer_list<CbCarrierBoardRelay::Contactor> {CbCarrierBoardRelay::Contactor::Contactor1},
                "Contactor 1");
            auto secondary = std::make_unique<CbCarrierBoardRelay>(
                this->controller,
                std::initializer_list<CbCarrierBoardRelay::Contactor> {CbCarrierBoardRelay::Contactor::Contactor2},
                "Contactor 2", true);

            this->contactor_controller = std::make_unique<CbContactorControlSimultaneous>(
                std::move(primary), this->config.contactor_1_feedback_type, std::move(secondary),
                this->config.contactor_2_feedback_type, "DC+", "DC-");
        }
    } else {
        // AC mode

        if (this->config.switch_3ph1ph_wiring == "none") {
            // per definition we use the contactor 1 path as primary/only relay for AC
            auto relay = std::make_unique<CbCarrierBoardRelay>(
                this->controller,
                std::initializer_list<CbCarrierBoardRelay::Contactor> {CbCarrierBoardRelay::Contactor::Contactor1},
                "Contactor 1");

            this->contactor_controller =
                std::make_unique<CbContactorControlSimple>(std::move(relay), this->config.contactor_1_feedback_type);

        } else if (this->config.switch_3ph1ph_wiring == "serial") {
            // per definition we use the contactor 1 as primary relay (all phases)
            // and contactor 2 as secondary relay (phases 2 and 3)
            auto primary = std::make_unique<CbCarrierBoardRelay>(
                this->controller,
                std::initializer_list<CbCarrierBoardRelay::Contactor> {CbCarrierBoardRelay::Contactor::Contactor1},
                "Contactor 1");
            auto secondary = std::make_unique<CbCarrierBoardRelay>(
                this->controller,
                std::initializer_list<CbCarrierBoardRelay::Contactor> {CbCarrierBoardRelay::Contactor::Contactor2},
                "Contactor 2");

            this->contactor_controller = std::make_unique<CbContactorControlSerial>(
                std::move(primary), this->config.contactor_1_feedback_type, std::move(secondary),
                this->config.contactor_2_feedback_type);

        } else if (this->config.switch_3ph1ph_wiring == "simultaneous") {
            // here we assume that primary switches one phase and neutral, the
            // secondary the remaining other phases
            auto primary = std::make_unique<CbCarrierBoardRelay>(
                this->controller,
                std::initializer_list<CbCarrierBoardRelay::Contactor> {CbCarrierBoardRelay::Contactor::Contactor1},
                "Contactor 1");
            auto secondary = std::make_unique<CbCarrierBoardRelay>(
                this->controller,
                std::initializer_list<CbCarrierBoardRelay::Contactor> {CbCarrierBoardRelay::Contactor::Contactor2},
                "Contactor 2", true);

            this->contactor_controller = std::make_unique<CbContactorControlSimultaneous>(
                std::move(primary), this->config.contactor_1_feedback_type, std::move(secondary),
                this->config.contactor_2_feedback_type);

        } else if (this->config.switch_3ph1ph_wiring == "mutual") {
            // per definition we use the relay 1 as primary relay (3ph) and relay 2 as secondary relay (1ph)
            auto r_3ph = std::make_unique<CbCarrierBoardRelay>(
                this->controller,
                std::initializer_list<CbCarrierBoardRelay::Contactor> {CbCarrierBoardRelay::Contactor::Contactor1},
                "Contactor 1");
            auto r_1ph = std::make_unique<CbCarrierBoardRelay>(
                this->controller,
                std::initializer_list<CbCarrierBoardRelay::Contactor> {CbCarrierBoardRelay::Contactor::Contactor2},
                "Contactor 2");

            this->contactor_controller =
                std::make_unique<CbContactorControlMutual>(std::move(r_3ph), this->config.contactor_1_feedback_type,
                                                           std::move(r_1ph), this->config.contactor_2_feedback_type);
        }
    }

    // initialize the interfaces now
    invoke_init(*p_ac_rcd);
    invoke_init(*p_evse_board_support);
    invoke_init(*p_temperatures);
}

void CbChargeSOMDriver::ready() {
    invoke_ready(*p_ac_rcd);
    invoke_ready(*p_evse_board_support);
    invoke_ready(*p_temperatures);
}

} // namespace module
