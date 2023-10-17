// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "evse_board_supportImpl.hpp"

namespace module {
namespace evse_board_support {

void evse_board_supportImpl::init() {
}

void evse_board_supportImpl::ready() {
}

void evse_board_supportImpl::handle_setup(bool& three_phases, bool& has_ventilation, std::string& country_code) {
    // your code for cmd setup goes here
}

types::evse_board_support::HardwareCapabilities evse_board_supportImpl::handle_get_hw_capabilities() {
    // your code for cmd get_hw_capabilities goes here
    return {};
}

void evse_board_supportImpl::handle_enable(bool& value) {
    // your code for cmd enable goes here
}

void evse_board_supportImpl::handle_pwm_on(double& value) {
    // your code for cmd pwm_on goes here
}

void evse_board_supportImpl::handle_pwm_off() {
    // your code for cmd pwm_off goes here
}

void evse_board_supportImpl::handle_pwm_F() {
    // your code for cmd pwm_F goes here
}

void evse_board_supportImpl::handle_allow_power_on(types::evse_board_support::PowerOnOff& value) {
    // your code for cmd allow_power_on goes here
}

void evse_board_supportImpl::handle_ac_switch_three_phases_while_charging(bool& value) {
    // your code for cmd ac_switch_three_phases_while_charging goes here
}

void evse_board_supportImpl::handle_evse_replug(int& value) {
    // your code for cmd evse_replug goes here
}

types::board_support_common::ProximityPilot evse_board_supportImpl::handle_ac_read_pp_ampacity() {
    // your code for cmd ac_read_pp_ampacity goes here
    return {};
}

void evse_board_supportImpl::handle_ac_set_overcurrent_limit_A(double& value) {
    // your code for cmd ac_set_overcurrent_limit_A goes here
}

} // namespace evse_board_support
} // namespace module
