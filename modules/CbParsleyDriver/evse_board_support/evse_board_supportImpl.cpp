// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include <chrono>
#include <iomanip>
#include <stdexcept>
#include <fmt/core.h>
#include <ra-utils/cb_protocol.h>
// B0 is defined in terminios.h for UART baudrate, but in CEState for MCS too - so undefine it before the inclusion
#undef B0
#include <generated/types/cb_board_support.hpp>
#include "evse_board_supportImpl.hpp"

const std::string safestate_active_error_subtype = "Safe State";

using namespace std::chrono_literals;

types::cb_board_support::CPState cestate_to_cpstate(const types::cb_board_support::CEState ce_state) {
    switch (ce_state) {
    case types::cb_board_support::CEState::PowerOn:
        return types::cb_board_support::CPState::PowerOn;
    case types::cb_board_support::CEState::A:
        return types::cb_board_support::CPState::A;
    case types::cb_board_support::CEState::B0:
        return types::cb_board_support::CPState::D;
    case types::cb_board_support::CEState::B:
        return types::cb_board_support::CPState::B;
    case types::cb_board_support::CEState::C:
        return types::cb_board_support::CPState::C;
    case types::cb_board_support::CEState::E:
        return types::cb_board_support::CPState::E;
    case types::cb_board_support::CEState::EC:
        return types::cb_board_support::CPState::F;
    default:
        return types::cb_board_support::CPState::PilotFault;
    }
}

types::board_support_common::BspEvent cpstate_to_bspevent(const types::cb_board_support::CPState& other) {
    switch (other) {
    case types::cb_board_support::CPState::A:
        return {types::board_support_common::Event::A};
    case types::cb_board_support::CPState::B:
        return {types::board_support_common::Event::B};
    case types::cb_board_support::CPState::C:
        return {types::board_support_common::Event::C};
    case types::cb_board_support::CPState::D:
        return {types::board_support_common::Event::D};
    case types::cb_board_support::CPState::E:
        return {types::board_support_common::Event::E};
    case types::cb_board_support::CPState::F:
        return {types::board_support_common::Event::F};
    default:
        throw std::runtime_error("Unable to map the value '" + cpstate_to_string(other) + "'.");
    }
}

namespace module {
namespace evse_board_support {

void evse_board_supportImpl::init() {
    // configure hardware capabilities: we used hard-coded values here since these values
    // are important for AC charging mostly and Charge Control Y is designed for DC only
    this->hw_capabilities.min_current_A_import = 6;
    this->hw_capabilities.max_current_A_import = 80;
    this->hw_capabilities.min_current_A_export = 6;
    this->hw_capabilities.max_current_A_export = 80;
    this->hw_capabilities.max_phase_count_import = 3;
    this->hw_capabilities.min_phase_count_import = 3;
    this->hw_capabilities.max_phase_count_export = 3;
    this->hw_capabilities.min_phase_count_export = 3;
    this->hw_capabilities.supports_changing_phases_during_charging = false;

    // we also lie about the real connector, but fixed cable is correct
    this->hw_capabilities.connector_type = types::evse_board_support::Connector_type::IEC62196Type2Cable;

    // register our callback handlers

    this->mod->controller.on_ce_change.connect([&](const types::cb_board_support::CEState& ce_state) {
        std::scoped_lock lock(this->cp_mutex);
        auto current_cp_state = cestate_to_cpstate(ce_state);

        // B0 is mapped to D; we need to ignore B0 and not forward it here
        if (current_cp_state == types::cb_board_support::CPState::D)
            return;

        EVLOG_info << "CP state change from " << this->cp_current_state << " to " << current_cp_state;
        this->cp_current_state = current_cp_state;

        // in case safety controller was in emergency state and EV is gone,
        // we have to reset safety controller with a disable -> enable toggle
        if (current_cp_state == types::cb_board_support::CPState::A and this->mod->controller.is_emergency()) {
            EVLOG_info << "recovering after safe state";

            // disable resets the controller
            this->mod->controller.disable();

            // enable starts UART frame processing again
            this->mod->controller.enable();
        }

        try {
            const types::board_support_common::BspEvent tmp = cpstate_to_bspevent(current_cp_state);
            this->publish_event(tmp);
        } catch (std::runtime_error& e) {
            // should never happen, when all invalid states are handled correctly
            EVLOG_warning << e.what();
        }
    });

    this->mod->controller.on_estop.connect([&](const enum cs2_estop_reason& reason) {
        if (reason == CS2_ESTOP_REASON_NO_STOP) {
            EVLOG_info << "Emergency Stop Cause disappeared";
            if (this->last_reported_fault.sub_type != safestate_active_error_subtype) {
                this->clear_error(this->last_reported_fault.type, this->last_reported_fault.sub_type);
                this->generic_fault_reported = false;
            }
        } else {
            std::string error_subtype = cb_proto_estop_reason_to_str(reason);
            std::ostringstream errmsg;

            errmsg << "Safety Controller issued an Emergency Stop due to " << reason;

            switch (reason) {
            case cs2_estop_reason::CS2_ESTOP_REASON_EMERGENCY_INPUT:
                EVLOG_error << errmsg.str() << ", raising MREC8EmergencyStop.";
                this->last_reported_fault = this->error_factory->create_error(
                    "evse_board_support/MREC8EmergencyStop", "", errmsg.str(), Everest::error::Severity::High);
                break;
            default:
                EVLOG_error << errmsg.str() << ", raising VendorError.";
                this->last_reported_fault = this->error_factory->create_error(
                    "evse_board_support/VendorError", error_subtype, errmsg.str(), Everest::error::Severity::High);
            }

            this->raise_error(this->last_reported_fault);
            this->generic_fault_reported = true;
        }
    });

    this->mod->controller.on_safestate_active.connect([&](const enum cs_safestate_active& state) {
        // Note: this handler is always called after the estop processing above is already done

        switch (state) {
        case cs_safestate_active::CS_SAFESTATE_ACTIVE_NORMAL:
            EVLOG_info << "Safety Controller back in normal mode";
            if (this->generic_fault_reported and
                (this->last_reported_fault.sub_type == safestate_active_error_subtype)) {
                this->clear_error(this->last_reported_fault.type, this->last_reported_fault.sub_type);
            }
            break;
        case cs_safestate_active::CS_SAFESTATE_ACTIVE_SAFESTATE:
            EVLOG_error << "Safety Controller entered safe state";
            // usually the estop handling above already raised the error, but in case
            // the safe state is triggered without an estop reason, we ensure here, that
            // EVerest is informed
            if (not this->generic_fault_reported) {
                this->last_reported_fault = this->error_factory->create_error(
                    "evse_board_support/VendorError", safestate_active_error_subtype,
                    "Safety MCU switched to safe state", Everest::error::Severity::High);
                this->raise_error(this->last_reported_fault);
                this->generic_fault_reported = true;
            }
            break;
        default:
            // all other values: we don't care
            ;
        }
    });

    this->mod->controller.on_errmsg.connect([&](bool is_active, unsigned int module, const std::string& module_str,
                                                unsigned int reason, const std::string& reason_str,
                                                unsigned int additional_data1, unsigned int additional_data2) {
        if (is_active) {
            std::string errmsg =
                fmt::format("{} ({:#06x}), {:#06x}, {:#06x}", reason_str, reason, additional_data1, additional_data2);

            EVLOG_warning << fmt::format("Safety Controller reported error: {} ({:#06x}), {}", module_str, module,
                                         errmsg);

            auto e = this->error_factory->create_error("evse_board_support/VendorWarning", module_str, errmsg,
                                                       Everest::error::Severity::High);

            this->raise_error(e);

        } else {
            std::string errmsg = fmt::format("{} ({:#06x})", reason_str, reason);

            EVLOG_info << fmt::format("Safety Controller cleared error: {} ({:#06x}), {}", module_str, module, errmsg);

            this->clear_error("evse_board_support/VendorWarning", module_str);
        }
    });
}

void evse_board_supportImpl::ready() {
    // the BSP must publish this variable at least once during start up
    this->publish_capabilities(this->hw_capabilities);
}

void evse_board_supportImpl::handle_enable(bool& value) {
    if (this->is_enabled.exchange(value) != value) {
        try {
            EVLOG_info << "handle_enable: " << std::boolalpha << value;

            // enable UART frame processing
            if (value)
                this->mod->controller.enable();
        } catch (std::exception& e) {
            EVLOG_error << e.what();
        }
    } else {
        EVLOG_debug << "handle_enable: " << std::boolalpha << value << " (suppressed)";
    }
}

void evse_board_supportImpl::handle_pwm_on(double& value) {
    EVLOG_info << "handle_pwm_on: setting new duty cycle of " << std::fixed << std::setprecision(1) << value
               << "% (ignored)";
}

void evse_board_supportImpl::handle_pwm_off() {
    EVLOG_info << "handle_pwm_off: setting new duty cycle of 100.0% (ignored)";
}

void evse_board_supportImpl::handle_pwm_F() {
    std::scoped_lock lock(this->cp_mutex);
    try {
        EVLOG_info << "handle_pwm_F: generating CP state F (aka EC)";

        this->mod->controller.set_ec_state();
    } catch (std::exception& e) {
        EVLOG_error << e.what();
    }
}

void evse_board_supportImpl::handle_allow_power_on(types::evse_board_support::PowerOnOff& value) {
    // set the new state atomically
    bool previous_state = this->contactor_state.exchange(value.allow_power_on);

    // this method is called very often, even the contactor state is already matching the desired one
    // so let's use this as helper to control the log noise a little bit and whether we actually
    // need to report a BSP event
    bool state_change = value.allow_power_on != previous_state;

    if (state_change) {
        EVLOG_info << "handle_allow_power_on: request to " << (value.allow_power_on ? "CLOSE" : "OPEN")
                   << " the contactor (simulated)";

        // publish PowerOn or PowerOff event
        types::board_support_common::Event tmp_event = value.allow_power_on
                                                           ? types::board_support_common::Event::PowerOn
                                                           : types::board_support_common::Event::PowerOff;
        types::board_support_common::BspEvent tmp {tmp_event};
        this->publish_event(tmp);
    } else {
        EVLOG_debug << "handle_allow_power_on: request to " << (value.allow_power_on ? "CLOSE" : "OPEN")
                    << " the contactor (ignored)";
    }
}

void evse_board_supportImpl::handle_ac_switch_three_phases_while_charging(bool& value) {
    EVLOG_error << "handle_ac_switch_three_phases_while_charging: switching to " << (value ? "3-phase" : "1-phase")
                << " mode was requested, this is probably a configuration error.";
}

void evse_board_supportImpl::handle_evse_replug(int& value) {
    // your code for cmd evse_replug goes here
    (void)value;
}

void evse_board_supportImpl::handle_ac_set_overcurrent_limit_A(double& value) {
    // your code for cmd ac_set_overcurrent_limit_A goes here
    (void)value;
}

} // namespace evse_board_support
} // namespace module
