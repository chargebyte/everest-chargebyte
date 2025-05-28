// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <queue>
#include <stdexcept>
#include <string>
#include <sstream>
#include <limits>
#include <system_error>
#include <thread>
#include <gpiod.hpp>
#include <sigslot/signal.hpp>
#include <ra-utils/uart.h>
#include <ra-utils/cb_protocol.h>
#include <gpiodUtils.hpp>
#include <generated/types/cb_board_support.hpp>
#include "CbChargeSOM.hpp"

using namespace std::chrono_literals;

CbChargeSOM::CbChargeSOM() {
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

void CbChargeSOM::terminate()
{
    this->termination_requested = true;
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

    // call the notification signals asynchronously to the frame receiving to avoid stalling
    this->notify_thread = std::thread([&]() {
        enum cp_state previous_cp_state = CP_STATE_UNKNOWN;
        enum pp_state previous_pp_state = PP_STATE_MAX;

        while (!this->termination_requested) {
            // temp helper for our access functions
            struct safety_controller tmpctx;
            enum pp_state current_pp_state;

            // wait for changes
            std::unique_lock<std::mutex> lock(this->notify_mutex);
            this->notify_cv.wait(lock, [&]() { return !this->charge_state_changes.empty(); });

            // remove from queue
            tmpctx.charge_state = this->charge_state_changes.front();
            this->charge_state_changes.pop();

            // check for PP changes
            current_pp_state = cb_proto_get_pp_state(&tmpctx);
            if (current_pp_state != previous_pp_state) {
                this->on_pp_change(current_pp_state);
                previous_pp_state = current_pp_state;
            }

            // check for CP changes
            current_cp_state = cb_proto_get_cp_state(&tmpctx);
            if (current_cp_state != previous_cp_state) {
                this->on_cp_change(current_cp_state);
                previous_cp_state = current_cp_state ;
            }
        }
    });

    // launch processing of received frames
    this->rx_thread = std::thread([&]() {
        // used to detect changes
        uint64_t previous_charge_state = std::numeric_limits<uint64_t>::max();

        while (!this->termination_requested) {
            enum cb_uart_com com;
            uint64_t data;
            bool notify = true;
            size_t n;
            int rv;

            rv = cb_uart_recv(&this->uart, &com, &data);
            if (rv) {
                switch (errno) {
                case EBADMSG:
                    // error is already logged via library
                    // usually this should not happen since we have a synchronized startup
                    // via reset line, only CRC errors might happen, so for now: do nothing
                    break;
                default:
                    throw std::system_error(errno, std::generic_category(), "Failed to receive from safety controller");
                }
            }

            // ignore all unknown COM values
            if (!this->is_valid_rx_com(com))
                continue;

            // map to the correct lock and condition variable for signaling
            n = static_cast<std::size_t>(com);
            std::scoped_lock lock(this->ctx_mutexes[n]);

            switch (com) {
            case cb_uart_com::COM_CHARGE_STATE:
                this->ctx.charge_state = data;
                // check if the previous value is different
                notify = previous_charge_state != data;
                previous_charge_state = data;
                // on change -> put new value into queue for notify thread
                if (notify) {
                    std::scoped_lock notify_lock(this->notify_mutex);
                    this->charge_state_changes.push(data);
                    this->notify_cv.notify_one();
                }
                break;

            case cb_uart_com::COM_PT1000_STATE:
                this->ctx.pt1000 = data;
                // note: notifying is not strictly needed here since the
                // temperature interface polls in regular interval by itself
                // but it also does not hurt
                break;

            case cb_uart_com::COM_FW_VERSION:
                this->ctx.fw_version = data;
                cb_proto_set_fw_version_str(&this->ctx);
                break;

            case cb_uart_com::COM_GIT_HASH:
                this->ctx.git_hash = data;
                cb_proto_set_git_hash_str(&this->ctx);
                break;

            default:
                /* not yet implemented */
                ;
            }

            // awaken possible waiters
            if (notify) {
                this->rx_cv[n].notify_all();
            }
        }
    });

    // release reset to start safety controller
    this->set_mcu_reset(false);

    // start sending periodic frames
    this->tx_thread = std::thread([&]() {
        std::chrono::milliseconds interval(CB_PROTO_CHARGE_CONTROL_INTERVAL);

        while (!this->termination_requested) {
            this->send_charge_control();
            std::this_thread::sleep_for(interval);
        }
    });

    // query firmware version and Git hash
    if (this->send_inquiry_and_wait(COM_FW_VERSION) or
        this->send_inquiry_and_wait(COM_GIT_HASH)) {
        throw std::runtime_error("Could not determined safety controller firmware information.");
    }

    this->fw_info = std::string(this->ctx.fw_version_str) + " (" +
                    cb_proto_fw_platform_type_to_str(cb_proto_fw_get_platform_type(&this->ctx)) + ", " +
                    cb_proto_fw_application_type_to_str(cb_proto_fw_get_application_type(&this->ctx)) + ", " +
                    this->ctx.git_hash_str + ")";
}

void CbChargeSOM::send_charge_control() {
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_CHARGE_CONTROL);
    std::scoped_lock lock(this->tx_mutex, this->ctx_mutexes[n]);

    if (cb_uart_send(&this->uart, COM_CHARGE_CONTROL, this->ctx.charge_control))
        throw std::system_error(errno, std::generic_category(), "Error while sending charge control frame");
}

bool CbChargeSOM::is_valid_rx_com(enum cb_uart_com com) {
    switch (com) {
    case COM_CHARGE_STATE:
    case COM_PT1000_STATE:
    case COM_FW_VERSION:
    case COM_GIT_HASH:
        return true;
    default:
        return false;
    }
}

void CbChargeSOM::send_inquiry(enum cb_uart_com com) {
    std::scoped_lock lock(this->tx_mutex);

    if (cb_send_uart_inquiry(&this->uart, com)) {
        throw std::system_error(errno, std::generic_category(),
                                std::string("Error while sending inquiry frame for '") + cb_uart_com_to_str(com) + "'");
    }
}

bool CbChargeSOM::send_inquiry_and_wait(enum cb_uart_com com) {
    size_t n = static_cast<std::size_t>(com);

    // ensure that only one inquiry is running
    std::scoped_lock inquiry_lock(this->inquiry_mutex);

    // acquire this mutex now so that we can ensure that we don't miss any
    // update to the response field in `ctx`
    std::unique_lock<std::mutex> lock(this->ctx_mutexes[n]);

    this->send_inquiry(com);

    // we should received a response at least within 1s
    return this->rx_cv[n].wait_for(lock, 1s) == std::cv_status::timeout;
}

void CbChargeSOM::reset() {
    std::scoped_lock lock(this->tx_mutex);

    this->set_mcu_reset(true);

    std::this_thread::sleep_for(this->mcu_reset_duration);

    if (uart_flush_input(&this->uart)) {
        throw std::system_error(errno, std::generic_category(),
                                "Failed to flush input data on '" + this->serial_port + "'");
    }

    this->set_mcu_reset(false);
}

void CbChargeSOM::set_mcu_reset(bool active) {
    this->mcu_reset->set_value(this->mcu_reset->offsets()[0],
                               active ? gpiod::line::value::ACTIVE : gpiod::line::value::INACTIVE);

    // when releasing the reset, wait until safety controller is capable to handle UART frames again
    if (not active) {
        std::this_thread::sleep_for(std::chrono::milliseconds(CB_PROTO_STARTUP_DELAY));
    }
}

unsigned int CbChargeSOM::get_temperature_channels() const {
    return CB_PROTO_MAX_PT1000S;
}

bool CbChargeSOM::is_temperature_enabled(unsigned int channel) {
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_PT1000_STATE);
    std::scoped_lock lock(this->ctx_mutexes[n]);

    return cb_proto_pt1000_is_active(&this->ctx, channel);
}

bool CbChargeSOM::is_temperature_valid(unsigned int channel) {
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_PT1000_STATE);
    std::scoped_lock lock(this->ctx_mutexes[n]);

    return !(cb_proto_pt1000_get_errors(&this->ctx, channel) & PT1000_SELFTEST_FAILED);
}

float CbChargeSOM::get_temperature(unsigned int channel) {
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_PT1000_STATE);
    std::scoped_lock lock(this->ctx_mutexes[n]);

    return cb_proto_pt1000_get_temp(&this->ctx, channel);
}

const std::string& CbChargeSOM::get_fw_info() const {
    return this->fw_info;
}

types::board_support_common::Ampacity CbChargeSOM::pp_state_to_ampacity(enum pp_state pp_state ) {
    // we map only the well-known states in this method - for all other a std::runtime_error is raised
    switch (pp_state) {
    case pp_state::PP_STATE_NO_CABLE:
        return types::board_support_common::Ampacity::None;

    case pp_state::PP_STATE_13A:
        return types::board_support_common::Ampacity::A_13;

    case pp_state::PP_STATE_20A:
        return types::board_support_common::Ampacity::A_20;

    case pp_state::PP_STATE_32A:
        return types::board_support_common::Ampacity::A_32;

    case pp_state::PP_STATE_63_70A:
        return types::board_support_common::Ampacity::A_63_3ph_70_1ph;

    default:
        throw std::runtime_error(
             "The measured voltage for the Proximity Pilot could not be mapped.");
    }
}

types::board_support_common::Ampacity CbChargeSOM::get_ampacity() {
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_CHARGE_STATE);
    std::scoped_lock lock(this->ctx_mutexes[n]);

    return this->pp_state_to_ampacity(cb_proto_get_pp_state(&this->ctx));
}

bool CbChargeSOM::is_emergency() {
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_CHARGE_STATE);
    std::scoped_lock lock(this->ctx_mutexes[n]);

    return cb_proto_estop_has_any_tripped(&this->ctx);
}

bool CbChargeSOM::get_contactor_state_no_lock() {
    unsigned int i;
    bool at_least_one_is_configured = false;
    bool target_state = false;
    bool actual_state = false;

    for (i = 0; i < CB_PROTO_MAX_CONTACTORS; ++i) {
        if (cb_proto_contactorN_is_enabled(&this->ctx, i)) {
            at_least_one_is_configured = true;

            // don't overwrite, but merge the state
            actual_state |= cb_proto_contactorN_is_closed(&this->ctx, i);
        }

        // fallback in the same loop in case no contactor is actually in use
        // don't overwrite, but merge the state
        target_state |= cb_proto_contactorN_get_target_state(&this->ctx, i);
    }

    if (at_least_one_is_configured)
        return actual_state;
    else
        return target_state;
}

bool CbChargeSOM::get_contactor_state() {
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_CHARGE_STATE);
    std::scoped_lock lock(this->ctx_mutexes[n]);

    return this->get_contactor_state_no_lock();
}

bool CbChargeSOM::switch_state(bool on) {
    // we need to take the lock to change the field
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_CHARGE_CONTROL);
    std::unique_lock<std::mutex> cc_lock(this->ctx_mutexes[n]);
    bool at_least_one_is_configured = false;
    unsigned int i;

    // we don't check here if the contactors are actually enabled
    for (i = 0; i < CB_PROTO_MAX_CONTACTORS; ++i) {
        cb_proto_contactorN_set_state(&this->ctx, i, on);

        if (cb_proto_contactorN_is_enabled(&this->ctx, i)) {
            at_least_one_is_configured = true;
        }
    }

    // but release it now so that sending can take the lock again
    cc_lock.unlock();

    this->send_charge_control();

    // if no real contactor is used, we simple report success back
    if (!at_least_one_is_configured)
        return true;

    // then we take the lock to access Charge State to check for success
    n = static_cast<std::size_t>(cb_uart_com::COM_CHARGE_STATE);
    std::unique_lock<std::mutex> cs_lock(this->ctx_mutexes[n]);

    // we should see the new value reflected within at max 1s (FIXME)
    return this->rx_cv[n].wait_for(cs_lock, 1s, [&] { return this->get_contactor_state_no_lock() == on; });
}

void CbChargeSOM::set_duty_cycle(unsigned int duty_cycle) {
    // we need to take the lock to change the field
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_CHARGE_CONTROL);
    std::unique_lock<std::mutex> cc_lock(this->ctx_mutexes[n]);

    cb_proto_set_duty_cycle(&this->ctx, duty_cycle);

    // but release it now so that sending can take the lock again
    cc_lock.unlock();

    this->send_charge_control();

    // then we take the lock to access Charge State to check for success
    n = static_cast<std::size_t>(cb_uart_com::COM_CHARGE_STATE);
    std::unique_lock<std::mutex> cs_lock(this->ctx_mutexes[n]);

    // we should see the new value reflected within at max 1s (FIXME)
    if (not this->rx_cv[n].wait_for(cs_lock, 1s, [&] {
        return cb_proto_get_actual_duty_cycle(&this->ctx) == duty_cycle; })) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << (duty_cycle / 10.0);

        throw std::runtime_error("Safety Controller did not accept the new duty cycle of " + oss.str() + "%");
    }
}

