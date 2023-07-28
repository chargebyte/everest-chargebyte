// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "board_support_ACImpl.hpp"

namespace module {
namespace board_support {

void board_support_ACImpl::init() {
}

void board_support_ACImpl::ready() {
}

void board_support_ACImpl::handle_setup(bool& three_phases, bool& has_ventilation, std::string& country_code,
                                        bool& rcd_enabled) {
    // your code for cmd setup goes here
}

types::board_support::HardwareCapabilities board_support_ACImpl::handle_get_hw_capabilities() {
    // your code for cmd get_hw_capabilities goes here
    return {};
}

void board_support_ACImpl::handle_enable(bool& value) {
    // your code for cmd enable goes here
}

void board_support_ACImpl::handle_pwm_on(double& value) {
    // your code for cmd pwm_on goes here
}

void board_support_ACImpl::handle_pwm_off() {
    // your code for cmd pwm_off goes here
}

void board_support_ACImpl::handle_pwm_F() {
    // your code for cmd pwm_F goes here
}

void board_support_ACImpl::handle_allow_power_on(bool& value) {
    // your code for cmd allow_power_on goes here
}

bool board_support_ACImpl::handle_force_unlock() {
    // your code for cmd force_unlock goes here
    return true;
}

void board_support_ACImpl::handle_switch_three_phases_while_charging(bool& value) {
    // your code for cmd switch_three_phases_while_charging goes here
}

void board_support_ACImpl::handle_evse_replug(int& value) {
    // your code for cmd evse_replug goes here
}

double board_support_ACImpl::handle_read_pp_ampacity() {
    // your code for cmd read_pp_ampacity goes here
    return 3.14;
}

} // namespace board_support
} // namespace module
