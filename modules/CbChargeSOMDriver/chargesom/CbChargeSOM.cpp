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
// B0 is defined in terminios.h for UART baudrate, but in CEState for MCS too - so undefine it before the inclusion
#undef B0
#include <generated/types/cb_board_support.hpp>
#include "CbChargeSOM.hpp"
#include <everest/logging.hpp>

using namespace std::chrono_literals;

std::ostream& operator<<(std::ostream& os, enum cp_state state) {
    return os << cb_proto_cp_state_to_str(state);
}

std::ostream& operator<<(std::ostream& os, enum pp_state state) {
    return os << cb_proto_pp_state_to_str(state);
}

std::ostream& operator<<(std::ostream& os, enum contactor_state state) {
    return os << cb_proto_contactor_state_to_str(state);
}

std::ostream& operator<<(std::ostream& os, enum estop_state state) {
    return os << cb_proto_estop_state_to_str(state);
}

std::ostream& operator<<(std::ostream& os, enum cs1_safestate_reason reason) {
    return os << cb_proto_safestate_reason_to_str(reason);
}

std::ostream& operator<<(std::ostream& os, enum cs_safestate_active state) {
    return os << cb_proto_safe_state_active_to_str(state);
}

CbChargeSOM::CbChargeSOM() {
    // clear the context structs before usage
    memset(&this->uart, 0, sizeof(this->uart));
    memset(&this->ctx, 0, sizeof(this->ctx));

    // set to invalid fd so that destructor knows whether it must close something
    this->uart.fd = -1;

    // we need a thread to call the notification signals asynchronously to the
    // frame receiving to avoid stalling and to not miss a single change
    this->notify_thread = std::thread([&]() {
        enum cp_state previous_cp_state = CP_STATE_MAX;
        enum pp_state previous_pp_state = PP_STATE_MAX;
        unsigned int previous_cp_errors = 0;
        bool previous_contactor_error[CB_PROTO_MAX_CONTACTORS] = {};
        enum cs1_safestate_reason previous_safestate_reason = CS1_SAFESTATE_REASON_MAX;
        enum cs_safestate_active previous_safestate_active = CS_SAFESTATE_ACTIVE_MAX;

        EVLOG_debug << "Notify Thread started";

        while (!this->termination_requested) {
            // temporary helper variables for our access functions
            struct safety_controller tmpctx;
            enum cp_state current_cp_state;
            enum pp_state current_pp_state;
            unsigned int current_cp_errors;
            bool current_contactor_error[CB_PROTO_MAX_CONTACTORS];
            enum cs1_safestate_reason current_safestate_reason;
            enum cs_safestate_active current_safestate_active;
            unsigned int i;

            // wait for changes
            std::unique_lock<std::mutex> lock(this->notify_mutex);
            this->notify_cv.wait(lock, [&]() {
                return (!this->charge_state_changes.empty() && this->evse_enabled) || this->termination_requested;
            });

            if (this->termination_requested)
                break;

            // remove from queue
            tmpctx.charge_state = this->charge_state_changes.front();
            this->charge_state_changes.pop();

            //
            // check for errors before doing the normal PP / CP state processing
            // this gives the upper layer the chance to report the error and/or set flags
            // before the actual low-level change is processed
            //

            // check for CP related errors
            current_cp_errors = cb_proto_get_cp_errors(&tmpctx);
            if (current_cp_errors != previous_cp_errors) {
                EVLOG_debug << "on_cp_error(0x" << std::hex << std::setw(2) << std::setfill('0') << previous_cp_errors
                            << " → "
                            << "0x" << std::hex << std::setw(2) << std::setfill('0') << current_cp_errors << ")";
                this->on_cp_error();
                previous_cp_errors = current_cp_errors;
            }

            // forward contactor errors
            for (i = 0; i < CB_PROTO_MAX_CONTACTORS; ++i) {
                current_contactor_error[i] =
                    cb_proto_contactorN_is_enabled(&tmpctx, i) && cb_proto_contactorN_has_error(&tmpctx, i);

                if (current_contactor_error[i] != previous_contactor_error[i]) {
                    std::string name = "Contactor " + std::to_string(i + 1);

                    EVLOG_debug << "on_contactor_error: " << i;
                    this->on_contactor_error(name, cb_proto_contactorN_get_target_state(&tmpctx, i),
                                             cb_proto_contactorN_is_closed(&tmpctx, i)
                                                 ? types::cb_board_support::ContactorState::Closed
                                                 : types::cb_board_support::ContactorState::Open);
                }
            }

            // check for changed safe state reason
            current_safestate_reason = cb_proto_get_safestate_reason(&tmpctx);
            if (current_safestate_reason != previous_safestate_reason) {
                if (previous_safestate_reason == CS1_SAFESTATE_REASON_MAX) {
                    EVLOG_debug << "on_estop(" << current_safestate_reason << ")"
                                << " [suppressed]";
                } else {
                    EVLOG_debug << "on_estop(" << current_safestate_reason << ")";
                    this->on_estop(current_safestate_reason);
                }
                previous_safestate_reason = current_safestate_reason;
            }

            // check for changed safe state active state
            current_safestate_active = cb_proto_get_safe_state_active(&tmpctx);
            if (current_safestate_active != previous_safestate_active) {
                // we suppress the normal/expected state change during boot into "normal" mode
                EVLOG_debug << "on_safestate_active(" << current_safestate_active << ")";
                if (previous_safestate_active == CS_SAFESTATE_ACTIVE_MAX &&
                    current_safestate_active == CS_SAFESTATE_ACTIVE_NORMAL) {
                    EVLOG_debug << "on_safestate_active(" << current_safestate_active << ")"
                                << " [suppressed]";
                } else {
                    EVLOG_debug << "on_safestate_active(" << current_safestate_active << ")";
                    this->on_safestate_active(current_safestate_active);
                }
                previous_safestate_active = current_safestate_active;
            }

            // check for PP changes
            current_pp_state = cb_proto_get_pp_state(&tmpctx);
            if (current_pp_state != previous_pp_state) {
                if (previous_pp_state != PP_STATE_MAX) {
                    if (this->is_pluggable) {
                        EVLOG_debug << "on_pp_change(" << previous_pp_state << " → " << current_pp_state << ")";
                        this->on_pp_change(current_pp_state);
                    } else {
                        EVLOG_debug << "on_pp_change(" << previous_pp_state << " → " << current_pp_state << ")"
                                    << " [suppressed, fixed cable]";
                    }
                } else {
                    EVLOG_debug << "on_pp_change(" << previous_pp_state << " → " << current_pp_state << ")"
                                << " [suppressed]";
                }
                previous_pp_state = current_pp_state;
            }

            // check for CP changes
            current_cp_state = cb_proto_get_cp_state(&tmpctx);
            if (current_cp_state != previous_cp_state) {
                // the integer value representation of both enum classes are the same
                types::cb_board_support::CPState new_cp_state =
                    static_cast<types::cb_board_support::CPState>(current_cp_state);

                if (previous_cp_state != CP_STATE_MAX) {
                    EVLOG_debug << "on_cp_change(" << previous_cp_state << " → " << current_cp_state << ")";
                    this->on_cp_change(new_cp_state);
                } else {
                    EVLOG_debug << "on_cp_change(" << previous_cp_state << " → " << current_cp_state << ")"
                                << " [suppressed]";
                }
                previous_cp_state = current_cp_state;
            }
        }

        EVLOG_debug << "Notify Thread terminated";
    });

    // launch processing of received frames
    this->rx_thread = std::thread([&]() {
        // used to detect changes
        uint64_t previous_charge_state = std::numeric_limits<uint64_t>::max();

        EVLOG_debug << "RX Thread started";

        while (!this->termination_requested) {
            enum cb_uart_com com;
            uint64_t payload;
            bool notify = true;
            size_t n;
            int rv;

            // in case we are holding the MCU (still) in reset, we can wait at least the startup
            // delay before we should receive anything
            if (this->is_mcu_reset_active) {
                std::this_thread::sleep_for(std::chrono::milliseconds(CB_PROTO_STARTUP_DELAY));
                continue;
            }

            // in case rx path is still disabled we loop without sleeping
            if (!this->rx_enabled)
                continue;

            // double check the file descriptor just to be sure
            if (this->uart.fd == -1)
                continue;

            rv = cb_uart_recv(&this->uart, &com, &payload);
            if (rv) {
                switch (errno) {
                case EBADMSG:
                    // error is already logged via library
                    // usually this should not happen since we have a synchronized startup
                    // via reset line, only CRC errors might happen, so for now: do nothing
                    break;
                case ETIMEDOUT:
                    // this is not an error in case we are (still) holding the MCU in reset
                    if (this->is_mcu_reset_active)
                        continue;
                    // it is also possible that we meanwhile disabled the rx path so we should
                    // not handle this as an error, too
                    if (!this->rx_enabled)
                        continue;
                    [[fallthrough]];
                default:
                    throw std::system_error(errno, std::generic_category(), "Failed to receive from safety controller");
                }
            }

            // ignore all unknown COM values
            // note: let's do _not_ log it to be upwards compatible since this could potentially flood the logs
            if (this->is_unexpected_rx_com(com))
                continue;

            // map to the correct lock and condition variable for signaling
            n = static_cast<std::size_t>(com);
            std::scoped_lock lock(this->ctx_mutexes[n]);

            switch (com) {
            case cb_uart_com::COM_CHARGE_STATE:
                this->ctx.charge_state = payload;
                // check if the previous value is different
                notify = previous_charge_state != payload;
                previous_charge_state = payload;
                // on change -> put new value into queue for notify thread
                if (notify) {
                    std::scoped_lock notify_lock(this->notify_mutex);
                    this->charge_state_changes.push(payload);
                    this->notify_cv.notify_one();
                }
                break;

            case cb_uart_com::COM_PT1000_STATE:
                this->ctx.pt1000 = payload;
                this->temperature_data_is_valid = true;
                // note: notifying is not strictly needed here since the
                // temperature interface polls in regular intervals by itself
                // but it also does not hurt
                break;

            case cb_uart_com::COM_FW_VERSION:
                this->ctx.fw_version = payload;
                cb_proto_set_fw_version_str(&this->ctx);
                break;

            case cb_uart_com::COM_GIT_HASH:
                this->ctx.git_hash = payload;
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

        EVLOG_debug << "RX Thread terminated";
    });

    // start sending periodic frames (becomes effective when the flag is set later)
    this->tx_thread = std::thread([&]() {
        const std::chrono::milliseconds interval(CB_PROTO_CHARGE_CONTROL_INTERVAL);
        bool start_notified = false;
        bool stop_notified = true;

        EVLOG_debug << "TX Thread started";

        while (!this->termination_requested) {
            if (this->tx_cc_enabled) {
                if (!start_notified) {
                    EVLOG_debug << "Starting sending of Charge Control frames";
                    start_notified = true;
                }
                stop_notified = false;

                this->send_charge_control();
            } else {
                start_notified = false;
                if (!stop_notified) {
                    EVLOG_debug << "Sending of Charge Control frames stopped";
                    stop_notified = true;
                }
            }

            if (!this->termination_requested) {
                std::this_thread::sleep_for(interval);
            }
        }

        EVLOG_debug << "TX Thread terminated";
    });
}

CbChargeSOM::~CbChargeSOM() {
    this->evse_enabled = false;
    this->tx_cc_enabled = false;

    if (this->notify_thread.joinable()) {
        this->notify_thread.join();
    }

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

void CbChargeSOM::terminate() {
    this->termination_requested = true;
}

void CbChargeSOM::init(const std::string& reset_gpio_line_name, bool reset_active_low, const std::string& serial_port,
                       bool is_pluggable, bool serial_trace) {
    int rv;

    // remember these settings
    this->is_pluggable = is_pluggable;
    this->serial_port = serial_port;

    // acquire the safety controller reset line
    // in case this fails, e.g. gpio line name is wrong, this will raise an std::runtime_error
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

    uart_trace(&this->uart, serial_trace);

    // release reset to start safety controller
    this->set_mcu_reset(false);

    // query firmware version and Git hash
    if (this->send_inquiry_and_wait(COM_FW_VERSION)) {
        throw std::runtime_error("Could not determine safety controller's firmware version.");
    }
    if (this->send_inquiry_and_wait(COM_GIT_HASH)) {
        throw std::runtime_error("Could not determine safety controller firmware's git hash.");
    }

    this->fw_info = std::string(this->ctx.fw_version_str) + " (g" + this->ctx.git_hash_str + ", " +
                    cb_proto_fw_platform_type_to_str(cb_proto_fw_get_platform_type(&this->ctx)) + ", " +
                    cb_proto_fw_application_type_to_str(cb_proto_fw_get_application_type(&this->ctx)) + ")";
}

void CbChargeSOM::enable() {
    EVLOG_debug << "request to enable the EVSE";

    // we hold the inquiry mutex here to ensure that nobody can switch
    // RX path enable in parallel
    std::scoped_lock inquiry_lock(this->inquiry_mutex);

    // we can directly return it was already enabled
    if (this->evse_enabled.exchange(true))
        return;

    // release reset to start safety controller
    this->set_mcu_reset(false);

    // start sending of periodic Charge Control frames
    this->tx_cc_enabled = true;

    // tell RX thread that we will receive frames now
    this->rx_enabled = true;
}

void CbChargeSOM::disable() {
    EVLOG_debug << "request to disable the EVSE";

    // we hold the inquiry mutex here to ensure that nobody can switch
    // RX path enable in parallel
    std::scoped_lock inquiry_lock(this->inquiry_mutex);

    // we can directly return it was already disabled
    if (!this->evse_enabled.exchange(false))
        return;

    // stop sending of periodic Charge Control frames
    this->tx_cc_enabled = false;

    // disable RX thread
    this->rx_enabled = false;

    // the RX thread has a long timeout -> we have to wait here the same amount of time
    // so that the RX thread can start aligned again
    std::this_thread::sleep_for(std::chrono::milliseconds(CB_UART_RECV_INTERVAL + CB_UART_RECV_INTERVAL / 2));

    // we must use `reset` here and not `set_mcu_reset` since we want to see state E
    // on CP line which is done by the safety controller until we start talking
    // to it again
    this->reset();
}

void CbChargeSOM::set_mcu_reset(bool active) {
    this->mcu_reset->set_value(this->mcu_reset->offsets()[0],
                               active ? gpiod::line::value::ACTIVE : gpiod::line::value::INACTIVE);
    this->is_mcu_reset_active = active;

    // when releasing the reset, wait until safety controller is capable to handle UART frames again
    if (not active) {
        std::this_thread::sleep_for(std::chrono::milliseconds(CB_PROTO_STARTUP_DELAY));
    }

    EVLOG_debug << "MCU reset line is now " << (active ? "ACTIVE" : "INACTIVE");
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

void CbChargeSOM::send_charge_control() {
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_CHARGE_CONTROL);
    std::scoped_lock lock(this->tx_mutex, this->ctx_mutexes[n]);

    if (cb_uart_send(&this->uart, COM_CHARGE_CONTROL, this->ctx.charge_control)) {
        throw std::system_error(errno, std::generic_category(), "Error while sending charge control frame");
    }
}

bool CbChargeSOM::is_unexpected_rx_com(enum cb_uart_com com) {
    switch (com) {
    case COM_CHARGE_STATE:
    case COM_PT1000_STATE:
    case COM_FW_VERSION:
    case COM_GIT_HASH:
        return false;
    default:
        return true;
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
    bool old_rx_enabled;
    bool rv;

    // ensure that only one inquiry is running
    std::scoped_lock inquiry_lock(this->inquiry_mutex);

    // acquire this mutex now so that we can ensure that we don't miss any
    // update to the response field in `ctx`
    std::unique_lock<std::mutex> lock(this->ctx_mutexes[n]);

    this->send_inquiry(com);

    // enable RX path
    old_rx_enabled = this->rx_enabled.exchange(true);

    // we should received a response at least within 1s
    rv = this->rx_cv[n].wait_for(lock, 1s) == std::cv_status::timeout;

    // in case RX path was disabled, disable it again
    if (!old_rx_enabled)
        this->rx_enabled.exchange(old_rx_enabled);

    return rv;
}

types::board_support_common::Ampacity CbChargeSOM::pp_state_to_ampacity(enum pp_state pp_state) {
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
        throw std::runtime_error("The measured voltage for the Proximity Pilot could not be mapped.");
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
    bool pp_error = false;

    if (this->is_pluggable) {
        // any state above this is considered an error for now
        pp_error = cb_proto_get_pp_state(&this->ctx) > PP_STATE_63_70A;
    }

    return pp_error or (cb_proto_get_cp_state(&this->ctx) == CP_STATE_INVALID) or
           (cb_proto_get_cp_errors(&this->ctx) != 0) or cb_proto_contactors_have_errors(&this->ctx) or
           cb_proto_estop_has_any_tripped(&this->ctx) or cb_proto_pt1000_have_errors(&this->ctx);
}

void CbChargeSOM::set_duty_cycle(unsigned int duty_cycle) {
    // we need to take the lock to change the field
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_CHARGE_CONTROL);
    std::unique_lock<std::mutex> cc_lock(this->ctx_mutexes[n]);

    cb_proto_set_pwm_active(&this->ctx, true);
    cb_proto_set_duty_cycle(&this->ctx, duty_cycle);

    // but release it now so that sending can take the lock again
    cc_lock.unlock();

    this->send_charge_control();

    // then we take the lock to access Charge State to check for success
    n = static_cast<std::size_t>(cb_uart_com::COM_CHARGE_STATE);
    std::unique_lock<std::mutex> cs_lock(this->ctx_mutexes[n]);

    // we should see the new value reflected within at max 1s (FIXME)
    if (not this->rx_cv[n].wait_for(cs_lock, 1s,
                                    [&] { return cb_proto_get_actual_duty_cycle(&this->ctx) == duty_cycle; })) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << (duty_cycle / 10.0);

        throw std::runtime_error("Safety Controller did not accept the new duty cycle of " + oss.str() + "%");
    }
}

unsigned int CbChargeSOM::get_duty_cycle() {
    // we need to take the lock to read the field
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_CHARGE_STATE);
    std::unique_lock<std::mutex> cc_lock(this->ctx_mutexes[n]);

    return cb_proto_get_actual_duty_cycle(&this->ctx);
}

bool CbChargeSOM::get_diode_fault() {
    // we need to take the lock to read the field
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_CHARGE_STATE);
    std::unique_lock<std::mutex> cc_lock(this->ctx_mutexes[n]);

    return cb_proto_is_diode_fault(&this->ctx);
}

bool CbChargeSOM::get_cp_short_circuit() {
    // we need to take the lock to read the field
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_CHARGE_STATE);
    std::unique_lock<std::mutex> cc_lock(this->ctx_mutexes[n]);

    return cb_proto_is_cp_short_circuit(&this->ctx);
}

bool CbChargeSOM::switch_state(bool on) {
    // we need to take the lock to change the field
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_CHARGE_CONTROL);
    std::unique_lock<std::mutex> cc_lock(this->ctx_mutexes[n]);
    bool at_least_one_is_configured = false;
    unsigned int i;

    for (i = 0; i < CB_PROTO_MAX_CONTACTORS; ++i) {
        // always remember the target state - without check whether it is configured at all
        cb_proto_contactorN_set_state(&this->ctx, i, on);
        if (cb_proto_contactorN_is_enabled(&this->ctx, i)) {
            at_least_one_is_configured = true;
        }
    }

    // but release it now so that sending can take the lock again
    cc_lock.unlock();

    this->send_charge_control();

    // if no real contactor is used, we simply report success back
    if (!at_least_one_is_configured)
        return true;

    // then we take the lock to access Charge State to check for success
    n = static_cast<std::size_t>(cb_uart_com::COM_CHARGE_STATE);
    std::unique_lock<std::mutex> cs_lock(this->ctx_mutexes[n]);

    // we should see the new value reflected within at max 1s (FIXME)
    return this->rx_cv[n].wait_for(cs_lock, 1s, [&] { return this->get_contactor_state_no_lock() == on; });
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

unsigned int CbChargeSOM::get_temperature_channels() const {
    return CB_PROTO_MAX_PT1000S;
}

float CbChargeSOM::get_temperature(unsigned int channel) {
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_PT1000_STATE);
    std::scoped_lock lock(this->ctx_mutexes[n]);

    return cb_proto_pt1000_get_temp(&this->ctx, channel);
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

unsigned int CbChargeSOM::get_temperature_errors(unsigned int channel) {
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_PT1000_STATE);
    std::scoped_lock lock(this->ctx_mutexes[n]);

    return cb_proto_pt1000_get_errors(&this->ctx, channel);
}

const std::string& CbChargeSOM::get_fw_info() const {
    return this->fw_info;
}
