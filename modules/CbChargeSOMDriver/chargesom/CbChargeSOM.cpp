// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <system_error>
#include <thread>
#include <generated/types/cb_board_support.hpp>
#include "CbChargeSOM.hpp"
#include "uart.h"
#include "cb_protocol.h"

using namespace std::chrono_literals;

CbChargeSOM::CbChargeSOM(void) {
    // clear the context struct before usage
    memset(&this->ctx, 0, sizeof(this->ctx));

    // set to invalid fd so that destructor knows whether it must close something
    this->ctx.uart.fd = -1;
}

CbChargeSOM::~CbChargeSOM() {
    if (this->ctx.uart.fd != -1) {
        uart_close(&this->ctx.uart);
    }
}

void CbChargeSOM::init(const std::string& serial_port, bool is_pluggable) {
    int rv;

    // remember this setting
    this->is_pluggable = is_pluggable;

    // open the configured device with well-known baudrate
    rv = uart_open(&this->ctx.uart, serial_port.c_str(), 115200);
    if (rv) {
        throw std::system_error(errno, std::generic_category(), "Failed to open '" + serial_port + "'");
    }
}

std::chrono::milliseconds CbChargeSOM::get_recovery_delay_ms() {
    return this->recovery_delay_ms;
}

void CbChargeSOM::cp_get_values(int& positive_value, int& negative_value) {
    // this method must be called with `ctx_mutex` already held
    positive_value = 1000 * (3.3 * this->ctx.data.cp_pos_peak_det * (47.0 + 150.0)) / (4096 * 47.0);

    negative_value = 1000 * (3.3 * this->ctx.data.cp_neg_peak_det * 150.0) / (4096 * -36.0);

    // FIXME values are not calibrated yet, so let's use static, manual offsets
    positive_value += 350; // mV
    negative_value -= 300; // mV
}

types::cb_board_support::CPState CbChargeSOM::cp_voltage_to_state(int voltage,
                                                                  types::cb_board_support::CPState previous_state) const {
    // The following thresholds mostly based on IEC 61851-1
    // Table A.4 System states detected by the charging station.
    if (voltage > 13000 /* mV */) /* > 13 V */
        return types::cb_board_support::CPState::PilotFault;

    if (voltage >= 11000 /* mV */) /* 11 V <= x <= 13 V */
        return types::cb_board_support::CPState::A;

    if (voltage >= 10000 /* mV */) { /* 10 V <= x < 11 V */
        switch (previous_state) {
        case types::cb_board_support::CPState::A:
            return types::cb_board_support::CPState::A;
        default:
            return types::cb_board_support::CPState::B;
        }
    }

    if (voltage >= 8000 /* mV */) /* 8 V <= x < 10 V */
        return types::cb_board_support::CPState::B;

    if (voltage >= 7000 /* mV */) { /* 7 V <= x < 8 V */
        switch (previous_state) {
        case types::cb_board_support::CPState::A:
        case types::cb_board_support::CPState::B:
            return types::cb_board_support::CPState::B;
        default:
            return types::cb_board_support::CPState::C;
        }
    }

    if (voltage >= 5000 /* mV */) /* 5 V <= x < 7 V */
        return types::cb_board_support::CPState::C;

    if (voltage >= 4000 /* mV */) { /* 4 V <= x < 5 V */
        switch (previous_state) {
        case types::cb_board_support::CPState::A:
        case types::cb_board_support::CPState::B:
        case types::cb_board_support::CPState::C:
            return types::cb_board_support::CPState::C;
        default:
            return types::cb_board_support::CPState::D;
        }
    }

    if (voltage >= 2000 /* mV */) /* 2 V <= x < 4 V */
        return types::cb_board_support::CPState::D;

    if (voltage >= 1000 /* mV */) { /* 1 V <= x < 2 V */
        switch (previous_state) {
        case types::cb_board_support::CPState::A:
        case types::cb_board_support::CPState::B:
        case types::cb_board_support::CPState::C:
        case types::cb_board_support::CPState::D:
            return types::cb_board_support::CPState::D;
        default:
            return types::cb_board_support::CPState::E;
        }
    }

    if (voltage >= -1000 /* mV */) /* -1 V <= x < 1 V */
        return types::cb_board_support::CPState::E;

    if (voltage >= -2000 /* mV */) { /* -2 V <= x < -1 V (not standard) */
        switch (previous_state) {
        case types::cb_board_support::CPState::A:
        case types::cb_board_support::CPState::B:
        case types::cb_board_support::CPState::C:
        case types::cb_board_support::CPState::D:
        case types::cb_board_support::CPState::E:
            return types::cb_board_support::CPState::E;
        default:
            return types::cb_board_support::CPState::PilotFault;
        }
    }

    if (voltage >= -10000 /* mV */) /* -10 V <= x < -2 V */
        return types::cb_board_support::CPState::PilotFault;

    if (voltage >= -11000 /* mV */) { /* -11 V <= x < -10 V (not standard) */
        return previous_state == types::cb_board_support::CPState::F ? types::cb_board_support::CPState::F
                                                                     : types::cb_board_support::CPState::PilotFault;
    }

    if (voltage >= -13000 /* mV */) /* -13 V <= x < -11 V */
        return types::cb_board_support::CPState::F;

    return types::cb_board_support::CPState::PilotFault; /* < -13 V */
}

double CbChargeSOM::cp_get_duty_cycle() const {
    // this method must be called with `ctx_mutex` already held

    return this->ctx.data.duty_cycle;
}

void CbChargeSOM::cp_set_duty_cycle(double duty_cycle) {
    std::scoped_lock lock(this->ctx_mutex);

    this->ctx.data.duty_cycle = floorl(duty_cycle);

    this->sync_with_hw();
}

bool CbChargeSOM::cp_is_nominal_duty_cycle() const {
    // this method must be called with `ctx_mutex` already held

    return 0 < this->ctx.data.duty_cycle && this->ctx.data.duty_cycle < 100;
}

bool CbChargeSOM::cp_is_enabled() {
    std::scoped_lock lock(this->ctx_mutex);

    // FIXME
    return true;
}

void CbChargeSOM::cp_disable() {
    std::scoped_lock lock(this->ctx_mutex);

    // FIXME it is unclear atm if the current safety controller fw can disable CP
    // so let's signal 100% for now
    this->ctx.data.duty_cycle = 100;

    this->sync_with_hw();
}

types::board_support_common::Ampacity CbChargeSOM::get_ampacity(int& voltage) {
    // FIXME
    voltage = this->ctx.data.pp_value * 3300 / 4096;

    // map the measured value

    if (voltage >= 3100)
        return types::board_support_common::Ampacity::None; // no cable connected

    if (voltage >= 2500)
        return types::board_support_common::Ampacity::A_13;

    if (voltage >= 2000)
        return types::board_support_common::Ampacity::A_20;

    if (voltage >= 1200)
        return types::board_support_common::Ampacity::A_32;

    if (voltage >= 700)
        return types::board_support_common::Ampacity::A_63_3ph_70_1ph;

    throw std::underflow_error(
        "The measured voltage for the Proximity Pilot is out-of-range (U_PP: " + std::to_string(voltage) + " mV).");
}

void CbChargeSOM::set_allow_power_on(bool allow_power_on) {
    std::scoped_lock lock(this->ctx_mutex);

    // repeat requesting the switch change until we succeed (infinitely for the moment)
    do {
        this->ctx.data.hvsw1_hs = allow_power_on;

        this->sync_with_hw();

    } while (this->ctx.data.hvsw1_hs != allow_power_on);
}

void CbChargeSOM::sync() {
    std::scoped_lock lock(this->ctx_mutex);
    this->sync_with_hw();

    if (this->is_pluggable) {
        // we first signal a potential PP state change to mimic the physical environment
        // (i.e. PP pin is slightly longer)
        this->signal_pp_state_change();
    }

    // then signal a CP state change
    this->signal_cp_state_change();
}

void CbChargeSOM::sync_with_hw() {
    int rv;

    // in case we want to monitor PP, we have to switch the PP pull-up on
    // and since the flag is also updated with the current hw state, we
    // always have to enforce that it is switched on - otherwise we might
    // actually switch it off again after reading it back as unset;
    // same applies to reverse direction
    this->ctx.data.pp_sae_iec = this->is_pluggable;

    rv = cb_single_run(&this->ctx);
    if (rv) {
        throw std::system_error(errno, std::generic_category(), "Communication with safety controller failed");
    }
}
