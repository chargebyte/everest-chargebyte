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
#include "CbParsley.hpp"
#include <everest/logging.hpp>

using namespace std::chrono_literals;

std::ostream& operator<<(std::ostream& os, enum cc2_ccs_ready state) {
    return os << cb_proto_ccs_ready_to_str(state);
}

std::ostream& operator<<(std::ostream& os, enum cs2_ce_state state) {
    return os << cb_proto_ce_state_to_str(state);
}

std::ostream& operator<<(std::ostream& os, enum cs2_id_state state) {
    return os << cb_proto_id_state_to_str(state);
}

std::ostream& operator<<(std::ostream& os, enum cs2_estop_reason state) {
    return os << cb_proto_estop_reason_to_str(state);
}

std::ostream& operator<<(std::ostream& os, enum cs_safestate_active state) {
    return os << cb_proto_safe_state_active_to_str(state);
}

types::cb_board_support::IDState id_state_to_IDState(enum cs2_id_state id_state) {
    // use a switch case because cs2_id_state values are not continuous
    switch (id_state) {
    case cs2_id_state::CS2_ID_STATE_UNKNOWN:
        return types::cb_board_support::IDState::PowerOn;
    case cs2_id_state::CS2_ID_STATE_NOT_CONNECTED:
        return types::cb_board_support::IDState::NotConnected;
    case cs2_id_state::CS2_ID_STATE_CONNECTED:
        return types::cb_board_support::IDState::Connected;
    case cs2_id_state::CS2_ID_STATE_INVALID:
        return types::cb_board_support::IDState::Invalid;
    default:
        throw std::runtime_error("Unable to map ID State value '" +
                                 std::to_string(static_cast<unsigned int>(id_state)) + "'.");
    }
}

types::cb_board_support::CEState ce_state_to_CEState(enum cs2_ce_state ce_state) {
    // use a switch case because cs2_ce_state values are not continuous
    switch (ce_state) {
    case cs2_ce_state::CS2_CE_STATE_UNKNOWN:
        return types::cb_board_support::CEState::PowerOn;
    case cs2_ce_state::CS2_CE_STATE_A:
        return types::cb_board_support::CEState::A;
    case cs2_ce_state::CS2_CE_STATE_B0:
        return types::cb_board_support::CEState::B0;
    case cs2_ce_state::CS2_CE_STATE_B:
        return types::cb_board_support::CEState::B;
    case cs2_ce_state::CS2_CE_STATE_C:
        return types::cb_board_support::CEState::C;
    case cs2_ce_state::CS2_CE_STATE_E:
        return types::cb_board_support::CEState::E;
    case cs2_ce_state::CS2_CE_STATE_EC:
        return types::cb_board_support::CEState::EC;
    case cs2_ce_state::CS2_CE_STATE_INVALID:
        return types::cb_board_support::CEState::Invalid;
    default:
        throw std::runtime_error("Unable to map CE State value '" +
                                 std::to_string(static_cast<unsigned int>(ce_state)) + "'.");
    }
}

CbParsley::CbParsley() {
    // clear the context structs before usage
    memset(&this->uart, 0, sizeof(this->uart));
    memset(&this->ctx, 0, sizeof(this->ctx));

    // set to invalid fd so that destructor knows whether it must close something
    this->uart.fd = -1;

    /* switch context to MCS mode */
    cb_proto_set_mcs_mode(&this->ctx, true);

    // we need a thread to call the notification signals asynchronously to the
    // frame receiving to avoid stalling and to not miss a single change
    this->notify_thread = std::thread([&]() {
        enum cs2_id_state previous_id_state = CS2_ID_STATE_MAX;
        enum cs2_ce_state previous_ce_state = CS2_CE_STATE_MAX;
        enum cs2_estop_reason previous_estop_reason = CS2_ESTOP_REASON_MAX;
        enum cs_safestate_active previous_safestate_active = CS_SAFESTATE_ACTIVE_MAX;

        EVLOG_debug << "Notify Thread started";

        while (!this->termination_requested) {
            // temporary helper variables for our access functions
            struct safety_controller tmpctx;
            enum cs2_id_state current_id_state;
            enum cs2_ce_state current_ce_state;
            enum cs2_estop_reason current_estop_reason;
            enum cs_safestate_active current_safestate_active;

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

            // check for changed ESTOP reason
            current_estop_reason = cb_proto_get_estop_reason(&tmpctx);
            if (current_estop_reason != previous_estop_reason) {
                if (previous_estop_reason == CS2_ESTOP_REASON_MAX) {
                    EVLOG_debug << "on_estop(" << current_estop_reason << ")"
                                << " [suppressed]";
                } else {
                    EVLOG_debug << "on_estop(" << current_estop_reason << ")";
                    this->on_estop(current_estop_reason);
                }
                previous_estop_reason = current_estop_reason;
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

            // check for ID changes
            current_id_state = cb_proto_get_id_state(&tmpctx);
            if (current_id_state != previous_id_state) {
                if (previous_id_state != CS2_ID_STATE_MAX) {
                    EVLOG_debug << "on_id_change(" << previous_id_state << " → " << current_id_state << ")";
                    auto id_state = id_state_to_IDState(current_id_state);
                    this->on_id_change(id_state);
                } else {
                    EVLOG_debug << "on_id_change(" << previous_id_state << " → " << current_id_state << ")"
                                << " [suppressed]";
                }
                previous_id_state = current_id_state;
            }

            // check for CE changes
            current_ce_state = cb_proto_get_ce_state(&tmpctx);
            if (current_ce_state != previous_ce_state) {
                if (previous_ce_state != CS2_CE_STATE_MAX) {
                    EVLOG_debug << "on_ce_change(" << previous_ce_state << " → " << current_ce_state << ")";
                    auto ce_state = ce_state_to_CEState(current_ce_state);
                    this->on_ce_change(ce_state);
                } else {
                    EVLOG_debug << "on_ce_change(" << previous_ce_state << " → " << current_ce_state << ")"
                                << " [suppressed]";
                }
                previous_ce_state = current_ce_state;
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
            if (this->is_unexpected_rx_com(com))
                continue;

            // map to the correct lock and condition variable for signaling
            n = static_cast<std::size_t>(com);
            std::scoped_lock lock(this->ctx_mutexes[n]);

            switch (com) {
            case cb_uart_com::COM_CHARGE_STATE_2:
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

CbParsley::~CbParsley() {
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

void CbParsley::terminate() {
    this->termination_requested = true;
}

void CbParsley::init(const std::string& reset_gpio_line_name, bool reset_active_low, const std::string& serial_port,
                     bool serial_trace) {
    int rv;

    // remember these settings
    this->serial_port = serial_port;

    // acquire the safety controller reset line
    // in case this fails, e.g. gpio line name is wrong, this will raise an std::runtime_error
    this->mcu_reset =
        std::make_unique<gpiod::line_request>(get_gpioline_by_name(reset_gpio_line_name, "CbParsleyDriver",
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
    if (this->send_inquiry_and_wait(COM_FW_VERSION) or this->send_inquiry_and_wait(COM_GIT_HASH)) {
        throw std::runtime_error("Could not determine safety controller firmware information.");
    }

    this->fw_info = std::string(this->ctx.fw_version_str) + " (g" + this->ctx.git_hash_str + ", " +
                    cb_proto_fw_platform_type_to_str(cb_proto_fw_get_platform_type(&this->ctx)) + ", " +
                    cb_proto_fw_application_type_to_str(cb_proto_fw_get_application_type(&this->ctx)) + ")";
}

void CbParsley::enable() {
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

void CbParsley::disable() {
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

    // we must use `reset` here and not `set_mcu_reset`
    this->reset();
}

void CbParsley::set_mcu_reset(bool active) {
    this->mcu_reset->set_value(this->mcu_reset->offsets()[0],
                               active ? gpiod::line::value::ACTIVE : gpiod::line::value::INACTIVE);
    this->is_mcu_reset_active = active;

    // when releasing the reset, wait until safety controller is capable to handle UART frames again
    if (not active) {
        std::this_thread::sleep_for(std::chrono::milliseconds(CB_PROTO_STARTUP_DELAY));
    }

    EVLOG_debug << "MCU reset line is now " << (active ? "ACTIVE" : "INACTIVE");
}

void CbParsley::reset() {
    std::scoped_lock lock(this->tx_mutex);

    this->set_mcu_reset(true);

    std::this_thread::sleep_for(this->mcu_reset_duration);

    if (uart_flush_input(&this->uart)) {
        throw std::system_error(errno, std::generic_category(),
                                "Failed to flush input data on '" + this->serial_port + "'");
    }

    this->set_mcu_reset(false);
}

void CbParsley::set_ccs_ready(bool enable) {
    // we need to take the lock to change the field
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_CHARGE_CONTROL_2);
    std::scoped_lock cc_lock(this->ctx_mutexes[n]);

    cb_proto_set_ccs_ready(&this->ctx, enable);

    // Note: we don't send this immediately out because we cannot guarantee that
    // we receive this request at a time when we actually want to communicate - so
    // we just store the flag and periodic communication will send it next time
}

void CbParsley::set_ec_state() {
    // we need to take the lock to change the field
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_CHARGE_CONTROL_2);
    std::unique_lock<std::mutex> cc_lock(this->ctx_mutexes[n]);

    cb_proto_set_estop(&this->ctx, true);

    // but release it now so that sending can take the lock again
    cc_lock.unlock();

    this->send_charge_control();

    // then we take the lock to access Charge State to check for success
    n = static_cast<std::size_t>(cb_uart_com::COM_CHARGE_STATE_2);
    std::unique_lock<std::mutex> cs_lock(this->ctx_mutexes[n]);

    // we should see the new value reflected within at max 1s (FIXME)
    if (not this->rx_cv[n].wait_for(cs_lock, 1s,
                                    [&] { return cb_proto_get_ce_state(&this->ctx) == CS2_CE_STATE_EC; })) {
        throw std::runtime_error("Safety Controller did not set EC state on CE");
    }
}

void CbParsley::send_charge_control() {
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_CHARGE_CONTROL_2);
    std::scoped_lock lock(this->tx_mutex, this->ctx_mutexes[n]);

    if (cb_uart_send(&this->uart, COM_CHARGE_CONTROL_2, this->ctx.charge_control)) {
        throw std::system_error(errno, std::generic_category(), "Error while sending charge control frame");
    }
}

bool CbParsley::is_unexpected_rx_com(enum cb_uart_com com) {
    switch (com) {
    case COM_CHARGE_STATE_2:
    case COM_PT1000_STATE:
    case COM_FW_VERSION:
    case COM_GIT_HASH:
        return false;
    default:
        return true;
    }
}

void CbParsley::send_inquiry(enum cb_uart_com com) {
    std::scoped_lock lock(this->tx_mutex);

    if (cb_send_uart_inquiry(&this->uart, com)) {
        throw std::system_error(errno, std::generic_category(),
                                std::string("Error while sending inquiry frame for '") + cb_uart_com_to_str(com) + "'");
    }
}

bool CbParsley::send_inquiry_and_wait(enum cb_uart_com com) {
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

bool CbParsley::is_emergency() {
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_CHARGE_STATE_2);
    std::scoped_lock lock(this->ctx_mutexes[n]);

    return (cb_proto_get_id_state(&this->ctx) == CS2_ID_STATE_INVALID) or
           (cb_proto_get_ce_state(&this->ctx) == CS2_CE_STATE_INVALID) or
           (cb_proto_get_estop_reason(&this->ctx) != CS2_ESTOP_REASON_NO_STOP) or
           (cb_proto_get_safe_state_active(&this->ctx) != CS_SAFESTATE_ACTIVE_NORMAL);
}

unsigned int CbParsley::get_temperature_channels() const {
    return CB_PROTO_MAX_PT1000S;
}

float CbParsley::get_temperature(unsigned int channel) {
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_PT1000_STATE);
    std::scoped_lock lock(this->ctx_mutexes[n]);

    return cb_proto_pt1000_get_temp(&this->ctx, channel);
}

bool CbParsley::is_temperature_enabled(unsigned int channel) {
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_PT1000_STATE);
    std::scoped_lock lock(this->ctx_mutexes[n]);

    return cb_proto_pt1000_is_active(&this->ctx, channel);
}

bool CbParsley::is_temperature_valid(unsigned int channel) {
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_PT1000_STATE);
    std::scoped_lock lock(this->ctx_mutexes[n]);

    return !(cb_proto_pt1000_get_errors(&this->ctx, channel) & PT1000_SELFTEST_FAILED);
}

unsigned int CbParsley::get_temperature_errors(unsigned int channel) {
    size_t n = static_cast<std::size_t>(cb_uart_com::COM_PT1000_STATE);
    std::scoped_lock lock(this->ctx_mutexes[n]);

    return cb_proto_pt1000_get_errors(&this->ctx, channel);
}

const std::string& CbParsley::get_fw_info() const {
    return this->fw_info;
}
