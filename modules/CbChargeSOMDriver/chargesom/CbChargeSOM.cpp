// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <gpiod.hpp>
#include <sigslot/signal.hpp>
#include <generated/types/cb_board_support.hpp>
#include <gpiodUtils.hpp>
#include "uart.h"
#include "cb_protocol.h"
#include "CbChargeSOM.hpp"

using namespace std::chrono_literals;

CbChargeSOM::CbChargeSOM(void) {
    // clear the context structs before usage
    memset(&this->uart, 0, sizeof(this->uart));
    memset(&this->ctx, 0, sizeof(this->ctx));

    // set to invalid fd so that destructor knows whether it must close something
    this->uart.fd = -1;
}

CbChargeSOM::~CbChargeSOM() {
    if (this->tx_thread.joinable()) {
        this->tx_thread.join();
    }

    if (this->rx_thread.joinable()) {
        this->rx_thread.join();
    }

    if (this->uart.fd != -1) {
        uart_close(&this->uart);
    }
}

void CbChargeSOM::init(const std::string& reset_gpio_line_name, bool reset_active_low, const std::string& serial_port,
                       bool is_pluggable) {
    int rv;

    // remember these settings
    this->is_pluggable = is_pluggable;
    this->serial_port = serial_port;

    // acquire the safety controller reset line
    this->mcu_reset =
        std::make_unique<gpiod::line_request>(get_gpioline_by_name(reset_gpio_line_name, "CbChargeSOMDriver",
                                                                   gpiod::line_settings()
                                                                       .set_direction(gpiod::line::direction::OUTPUT)
                                                                       .set_output_value(gpiod::line::value::ACTIVE)
                                                                       .set_active_low(reset_active_low)));

    // open the configured device with well-known baudrate
    rv = uart_open(&this->uart, serial_port.c_str(), 115200);
    if (rv) {
        throw std::system_error(errno, std::generic_category(), "Failed to open '" + this->serial_port + "'");
    }

    // we are holding the safety in reset and we just opened the UART interface
    // so this might not be necessary at all, but to be on the safe side
    rv = uart_flush_input(&this->uart);
    if (rv) {
        throw std::system_error(errno, std::generic_category(),
                                "Failed to flush input data on '" + this->serial_port + "'");
    }

    // launch processing of received frames
    this->rx_thread = std::thread([&]() {
        while (true) {
            enum cb_uart_com com;
            uint64_t data;
            int rv;

            rv = cb_uart_recv(&this->uart, &com, &data);
            if (rv) {
                switch (errno) {
                case EBADMSG:
                    // error is already logged via library
                    // usually this should not happen since we have a synchronized startup
                    // via reset line, only CRC errors might happen,
                    // so for now: do nothing
                    break;
                default:
                    throw std::system_error(errno, std::generic_category(), "Failed to receive from safety controller");
                }
            }

            switch (com) {
            case COM_CHARGE_STATE: {
                std::scoped_lock lock(this->charge_state_mutex);
                this->ctx.charge_state = data;
                break;
            }
            case COM_PT1000_STATE: {
                std::scoped_lock lock(this->pt1000_mutex);
                this->ctx.pt1000 = data;
                break;
            }
            case COM_FW_VERSION:
                // we don't need a mutex here since we access this field only once and under defined sequence
                this->ctx.fw_version = data;
                cb_proto_set_fw_version_str(&this->ctx);
                break;
            case COM_GIT_HASH:
                // we don't need a mutex here since we access this field only once and under defined sequence
                this->ctx.git_hash = data;
                cb_proto_set_git_hash_str(&this->ctx);
                break;
            default:
                /* not yet implemented */
            }

            // awaken possible waiters
            {
                std::scoped_lock lock(this->rx_mutex);
                this->latest_received_com = com;
            }
            this->rx_cv.notify_all();
        }
    });

    // release reset to start safety controller and give short time to startup:
    // we simply assume that we should receive a UART frame within 1s after releasing the reset line
    std::unique_lock<std::mutex> lock(this->rx_mutex);

    this->set_mcu_reset(false);

    if (!this->rx_cv.wait_for(lock, 1s, [&] { return this->latest_received_com == COM_CHARGE_STATE; })) {
        throw std::runtime_error("Safety controller did not startup.");
    }

    // query firmware version and Git hash
    if (this->send_inquiry_and_wait(COM_FW_VERSION) or this->send_inquiry_and_wait(COM_GIT_HASH)) {
        throw std::runtime_error("Could not determined safety controller firmware information.");
    }

    this->fw_info = std::string(this->ctx.fw_version_str) + " (" +
                    cb_proto_fw_platform_type_to_str(cb_proto_fw_get_platform_type(&this->ctx)) + ", " +
                    cb_proto_fw_application_type_to_str(cb_proto_fw_get_application_type(&this->ctx)) + ", " +
                    this->ctx.git_hash_str + ")";

    // start sending periodic frames
    this->tx_thread = std::thread([&]() {
        std::chrono::milliseconds interval(CHARGE_CONTROL_INTERVAL);

        while (true) {
            this->send_charge_control();
            std::this_thread::sleep_for(interval);
        }
    });
}

void CbChargeSOM::send_charge_control() {
    std::scoped_lock lock(this->tx_mutex, this->charge_control_mutex);

    if (cb_uart_send(&this->uart, COM_CHARGE_CONTROL, this->ctx.charge_control))
        throw std::system_error(errno, std::generic_category(), "Error while sending charge control frame");
}

void CbChargeSOM::send_inquiry(enum cb_uart_com com) {
    std::scoped_lock lock(this->tx_mutex);

    if (cb_send_uart_inquiry(&this->uart, com))
        throw std::system_error(errno, std::generic_category(),
                                std::string("Error while sending inquiry frame for '") + cb_uart_com_to_str(com) + "'");
}

bool CbChargeSOM::send_inquiry_and_wait(enum cb_uart_com com) {

    std::unique_lock<std::mutex> lock(this->rx_mutex);

    this->send_inquiry(com);

    // we should received a response at least within 1s
    return !this->rx_cv.wait_for(lock, 1s, [&] { return this->latest_received_com == com; });
}

void CbChargeSOM::reset() {
    std::scoped_lock lock(this->tx_mutex);

    this->set_mcu_reset(true);

    std::this_thread::sleep_for(this->mcu_reset_duration);

    if (uart_flush_input(&this->uart))
        throw std::system_error(errno, std::generic_category(),
                                "Failed to flush input data on '" + this->serial_port + "'");

    this->set_mcu_reset(false);
}

void CbChargeSOM::set_mcu_reset(bool active) {
    this->mcu_reset->set_value(this->mcu_reset->offsets()[0],
                               active ? gpiod::line::value::ACTIVE : gpiod::line::value::INACTIVE);
}

unsigned int CbChargeSOM::get_temperature_channels() const {
    return MAX_PT1000_CHANNELS;
}

bool CbChargeSOM::is_temperature_enabled(unsigned int channel) const {
    std::scoped_lock lock(this->pt1000_mutex);

    return cb_proto_pt1000_is_active(&this->ctx, channel);
}

bool CbChargeSOM::is_temperature_valid(unsigned int channel) const {
    std::scoped_lock lock(this->pt1000_mutex);

    return !(cb_proto_pt1000_get_errors(&this->ctx, channel) & PT1000_SELFTEST_FAILED);
}

float CbChargeSOM::get_temperature(unsigned int channel) const {
    std::scoped_lock lock(this->pt1000_mutex);

    return cb_proto_pt1000_get_temp(&this->ctx, channel);
}

const std::string& CbChargeSOM::get_fw_info() const {
    return this->fw_info;
}

enum pp_state CbChargeSOM::get_ampacity() {
    std::scoped_lock lock(this->charge_state_mutex);

    return cb_proto_get_pp_state(&this->ctx);
}

void CbChargeSOM::set_allow_power_on(bool allow_power_on) {
    std::scoped_lock lock(this->ctx_mutex);

    // repeat requesting the switch change until we succeed (infinitely for the moment)
    do {
        this->ctx.data.hvsw1_hs = allow_power_on;

        this->sync_with_hw();

    } while (this->ctx.data.hvsw1_hs != allow_power_on);
}
