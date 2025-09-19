// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include <chrono>
#include <iomanip>
#include <stdexcept>
#include <generated/types/cb_board_support.hpp>
#include <CPUtils.hpp>
#include "evse_board_supportImpl.hpp"

const std::string safestate_active_error_subtype = "Safe State";

using namespace std::chrono_literals;

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
    // configure hardware capabilities: use user-configurable settings for flexibility
    // but use the same value for import and export - there seems to be no reason for AC
    // that these values differ for import and export
    this->hw_capabilities.min_current_A_import = this->mod->config.min_current_A;
    this->hw_capabilities.max_current_A_import = this->mod->config.max_current_A;
    this->hw_capabilities.min_current_A_export = this->mod->config.min_current_A;
    this->hw_capabilities.max_current_A_export = this->mod->config.max_current_A;

    // the Charge SOM is currently intended for DC only, there is no support in the
    // safety controller firmware for AC nor phase count switching yet
    this->hw_capabilities.supports_changing_phases_during_charging = false;
    this->hw_capabilities.max_phase_count_import = 3;
    this->hw_capabilities.min_phase_count_import = 3;
    this->hw_capabilities.max_phase_count_export = 3;
    this->hw_capabilities.min_phase_count_export = 3;

    this->hw_capabilities.connector_type =
        types::evse_board_support::string_to_connector_type(this->mod->config.connector_type);

    // register our callback handlers

    this->mod->controller.on_pp_change.connect([&](const enum pp_state& new_pp_state) {
        std::scoped_lock lock(this->pp_mutex);

        try {
            // saved previous value
            types::board_support_common::Ampacity prev_value(this->pp_ampacity.ampacity);

            this->pp_ampacity.ampacity = this->mod->controller.pp_state_to_ampacity(new_pp_state);

            if (this->pp_ampacity.ampacity == types::board_support_common::Ampacity::None) {
                EVLOG_info << "PP noticed plug removal from socket";
            } else {
                EVLOG_info << "PP ampacity change from " << prev_value << " to " << this->pp_ampacity.ampacity;
            }

            // publish new value, upper layer should decide how to handle the change
            this->publish_ac_pp_ampacity(this->pp_ampacity);

            if (this->pp_ampacity.ampacity == types::board_support_common::Ampacity::None &&
                (this->cp_current_state == types::cb_board_support::CPState::C ||
                 this->cp_current_state == types::cb_board_support::CPState::D)) {
                // publish a ProximityFault
                Everest::error::Error error_object = this->error_factory->create_error(
                    "evse_board_support/MREC23ProximityFault", "", "Plug removed from socket during charge",
                    Everest::error::Severity::High);
                this->raise_error(error_object);
                this->pp_fault_reported = true;
            }

            if (this->pp_ampacity.ampacity != types::board_support_common::Ampacity::None && this->pp_fault_reported) {
                // clear a ProximityFault error on PP state change to a valid value but only if it exists
                this->clear_error("evse_board_support/MREC23ProximityFault");
                this->pp_fault_reported = false;
            }
        } catch (std::runtime_error& e) {
            if (!this->pp_fault_reported) {
                EVLOG_error << e.what();

                // publish a ProximityFault
                Everest::error::Error error_object = this->error_factory->create_error(
                    "evse_board_support/MREC23ProximityFault", "", e.what(), Everest::error::Severity::High);
                this->raise_error(error_object);

                this->pp_fault_reported = true;
            }
        }
    });

    this->mod->controller.on_cp_change.connect([&](const types::cb_board_support::CPState& current_cp_state) {
        std::scoped_lock lock(this->cp_mutex);

        // FIXME REVISIT
        // safety controller can report CP state unknown (which maps to our PowerOn here) in case
        // of e.g short-circuit (should be clarified with safety fw development whether better report E)
        if (current_cp_state == types::cb_board_support::CPState::PowerOn)
            return;

        // FIXME REVISIT special case: during power on filter out transient state E
        if (this->cp_current_state == types::cb_board_support::CPState::PowerOn &&
            current_cp_state == types::cb_board_support::CPState::E) {
            EVLOG_debug << "CP state change from " << this->cp_current_state << " to " << current_cp_state << ", "
                        << "PWM: " << std::fixed << std::setprecision(1)
                        << (this->mod->controller.get_duty_cycle() / 10.0) << "%"
                        << " [filtered]";
            return;
        }

        EVLOG_info << "CP state change from " << this->cp_current_state << " to " << current_cp_state << ", "
                   << "PWM: " << std::fixed << std::setprecision(1) << (this->mod->controller.get_duty_cycle() / 10.0)
                   << "%";

        // we can determine this directly from the currently seen value
        this->cp_errors.pilot_fault.is_active = current_cp_state == types::cb_board_support::CPState::PilotFault;

        CPUtils::process_everest_errors(*this, this->cp_errors.errors);

        this->cp_current_state = current_cp_state;

        if (current_cp_state == types::cb_board_support::CPState::PilotFault)
            return;

        try {
            const types::board_support_common::BspEvent tmp = cpstate_to_bspevent(current_cp_state);
            this->publish_event(tmp);
        } catch (std::runtime_error& e) {
            // should never happen, when all invalid states are handled correctly
            EVLOG_warning << e.what();
        }
    });

    this->mod->controller.on_cp_error.connect([&]() {
        std::scoped_lock lock(this->cp_mutex);

        // we can determine this directly from the currently seen values
        this->cp_errors.diode_fault.is_active = this->mod->controller.get_diode_fault();
        this->cp_errors.cp_short_fault.is_active = this->mod->controller.get_cp_short_circuit();

        CPUtils::process_everest_errors(*this, this->cp_errors.errors);

        // we only need to remember the PilotFault here, reset is done in `on_cp_change`
        if (this->cp_errors.diode_fault.is_active or this->cp_errors.cp_short_fault.is_active) {
            this->cp_current_state = types::cb_board_support::CPState::PilotFault;
        }
    });

    this->mod->controller.on_contactor_error.connect(
        [&](const std::string& source, bool desired_state, types::cb_board_support::ContactorState actual_state) {
            // An error occurred while switching - i.e. feedback does not match our expected new state.
            // This fault is critical as it might be a sign of a hardware issue, therefore
            // the fault will not be cleared to prevent further damage.
            std::ostringstream errmsg;
            errmsg << "Failed to " << (desired_state ? "CLOSE" : "OPEN") << " the " << source << ", it is still "
                   << actual_state;

            // raise a contactor fault if it was not done before
            if (!this->contactor_fault_reported.exchange(true)) {
                EVLOG_error << errmsg.str() << ", raising MREC17EVSEContactorFault.";

                Everest::error::Error error_object = this->error_factory->create_error(
                    "evse_board_support/MREC17EVSEContactorFault", "", errmsg.str(), Everest::error::Severity::High);
                this->raise_error(error_object);
            } else {
                EVLOG_error << errmsg.str();
            }
        });

    this->mod->controller.on_estop.connect([&](const enum cs1_safestate_reason& reason) {
        if (reason == CS1_SAFESTATE_REASON_NO_STOP) {
            EVLOG_info << "Emergency Stop Cause disappeared";
            if (this->last_reported_fault.sub_type != safestate_active_error_subtype) {
                this->clear_error(this->last_reported_fault.type, this->last_reported_fault.sub_type);
                this->generic_fault_reported = false;
            }
        } else {
            std::string error_subtype = cb_proto_safestate_reason_to_str(reason);
            std::ostringstream errmsg;

            errmsg << "Safety Controller issued an Emergency Stop due to " << reason;

            switch (reason) {
            case cs1_safestate_reason::CS1_SAFESTATE_REASON_PP_MALFUNCTION:
            case cs1_safestate_reason::CS1_SAFESTATE_REASON_CP_MALFUNCTION:
            case cs1_safestate_reason::CS1_SAFESTATE_REASON_CP_SHORT_CIRCUIT:
            case cs1_safestate_reason::CS1_SAFESTATE_REASON_CP_DIODE_FAULT:
            case cs1_safestate_reason::CS1_SAFESTATE_REASON_HV_SWITCH_MALFUNCTION:
                // these errors are already handled via other means
                return;
            case cs1_safestate_reason::CS1_SAFESTATE_REASON_EMERGENCY_INPUT_1:
            case cs1_safestate_reason::CS1_SAFESTATE_REASON_EMERGENCY_INPUT_2:
            case cs1_safestate_reason::CS1_SAFESTATE_REASON_EMERGENCY_INPUT_3:
                EVLOG_error << errmsg.str() << ", raising MREC8EmergencyStop.";
                this->last_reported_fault =
                    this->error_factory->create_error("evse_board_support/MREC8EmergencyStop", error_subtype,
                                                      errmsg.str(), Everest::error::Severity::High);
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
            std::ostringstream errmsg;
            errmsg << reason_str << " (" << std::showbase << std::setw(4) << std::setfill('0') << std::hex << reason
                   << "), " << std::showbase << std::setw(4) << std::setfill('0') << std::hex << additional_data1
                   << ", " << std::showbase << std::setw(4) << std::setfill('0') << std::hex << additional_data2;

            EVLOG_warning << "Safety Controller reported error: " << module_str << "(" << std::showbase << std::setw(4)
                          << std::setfill('0') << std::hex << module << "), " << errmsg.str();

            auto e = this->error_factory->create_error("evse_board_support/VendorWarning", module_str, errmsg.str(),
                                                       Everest::error::Severity::High);

            this->raise_error(e);

        } else {
            std::ostringstream errmsg;
            errmsg << reason_str << " (" << std::showbase << std::setw(4) << std::setfill('0') << std::hex << reason
                   << ")";

            EVLOG_info << "Safety Controller cleared error: " << module_str << "(" << std::showbase << std::setw(4)
                       << std::setfill('0') << std::hex << module << "), " << errmsg.str();

            this->clear_error("evse_board_support/VendorWarning", module_str);
        }
    });
}

void evse_board_supportImpl::ready() {
    // the BSP must publish this variable at least once during start up
    this->publish_capabilities(this->hw_capabilities);
}

void evse_board_supportImpl::handle_enable(bool& value) {
    try {
        // enable UART frame processing
        if (value)
            this->mod->controller.enable();

        // generate state A or state F
        unsigned int new_duty_cycle = value ? 1000 : 0;

        EVLOG_info << "handle_enable: Setting new duty cycle of " << std::fixed << std::setprecision(1)
                   << (new_duty_cycle / 10.0) << "%";
        this->mod->controller.set_duty_cycle(new_duty_cycle);

        this->is_enabled = value;
    } catch (std::exception& e) {
        EVLOG_error << e.what();
    }
}

void evse_board_supportImpl::handle_pwm_on(double& value) {
    try {
        unsigned int new_duty_cycle = static_cast<unsigned int>(value * 10.0);

        EVLOG_info << "handle_pwm_on: Setting new duty cycle of " << std::fixed << std::setprecision(1)
                   << (new_duty_cycle / 10.0) << "%";
        this->mod->controller.set_duty_cycle(new_duty_cycle);
    } catch (std::exception& e) {
        EVLOG_error << e.what();
    }
}

void evse_board_supportImpl::handle_pwm_off() {
    // in case safety controller was in emergency state, we have to reset it
    // with a disable -> enable toggle
    if (this->mod->controller.is_emergency()) {
        std::scoped_lock lock(this->cp_mutex);

        EVLOG_info << "handle_pwm_off: recovering after safe state";

        // disable resets the controller and goes shortly to state E
        this->mod->controller.disable();

        // reset the remembered state
        this->cp_current_state = types::cb_board_support::CPState::PowerOn;

        // enable starts UART frame processing again
        this->mod->controller.enable();
    }

    try {
        // generate state A
        unsigned int new_duty_cycle = 1000;

        EVLOG_info << "handle_pwm_off: Setting new duty cycle of " << std::fixed << std::setprecision(1)
                   << (new_duty_cycle / 10.0) << "%";
        this->mod->controller.set_duty_cycle(new_duty_cycle);
    } catch (std::exception& e) {
        EVLOG_error << e.what();
    }
}

void evse_board_supportImpl::handle_pwm_F() {
    try {
        // generate state F
        unsigned int new_duty_cycle = 0;

        EVLOG_info << "handle_pwm_F: Generating CP state F";

        this->mod->controller.set_duty_cycle(new_duty_cycle);
    } catch (std::exception& e) {
        EVLOG_error << e.what();
    }
}

void evse_board_supportImpl::handle_allow_power_on(types::evse_board_support::PowerOnOff& value) {
    // this method is called very often, even the contactor state is already matching the desired one
    // so let's use this as helper to control the log noise a little bit and whether we actually
    // need to report a BSP event
    bool state_change = value.allow_power_on != this->mod->controller.get_contactor_state();

    if (value.allow_power_on && this->cp_current_state == types::cb_board_support::CPState::PilotFault) {
        EVLOG_warning << "Power on rejected due to detected pilot fault";
        return;
    }

    if (value.allow_power_on && this->mod->controller.is_emergency()) {
        EVLOG_warning << "Power on rejected due to detected emergency state";
        return;
    }

    if (value.allow_power_on && this->contactor_fault_reported) {
        EVLOG_warning << "Power on rejected due to contactor fault present";
        return;
    }

    if (state_change)
        EVLOG_info << "handle_allow_power_on: request to " << (value.allow_power_on ? "CLOSE" : "OPEN")
                   << " the contactor";
    else
        EVLOG_debug << "handle_allow_power_on: request to " << (value.allow_power_on ? "CLOSE" : "OPEN")
                    << " the contactor";

    // exit early if we don't actually change the state
    if (!state_change) {
        EVLOG_info << "Current (unchanged) state: "
                   << (this->mod->controller.get_contactor_state() ? "CLOSED" : "OPEN");
        return;
    }

    if (this->mod->controller.switch_state(value.allow_power_on)) {
        EVLOG_info << "Current state: " << (this->mod->controller.get_contactor_state() ? "CLOSED" : "OPEN");

        // publish PowerOn or PowerOff event
        types::board_support_common::Event tmp_event = value.allow_power_on
                                                           ? types::board_support_common::Event::PowerOn
                                                           : types::board_support_common::Event::PowerOff;
        types::board_support_common::BspEvent tmp {tmp_event};
        this->publish_event(tmp);
    }
    // Note: errors while switching the contactor are reported and handled via on_error slot
}

void evse_board_supportImpl::handle_ac_switch_three_phases_while_charging(bool& value) {
    EVLOG_info << "handle_ac_switch_three_phases_while_charging: switching to " << (value ? "3-phase" : "1-phase")
               << " mode";
    (void)value;
}

void evse_board_supportImpl::handle_evse_replug(int& value) {
    // your code for cmd evse_replug goes here
    (void)value;
}

types::board_support_common::ProximityPilot evse_board_supportImpl::handle_ac_read_pp_ampacity() {
    std::scoped_lock lock(this->pp_mutex);

    // save old value and pre-init to None
    types::board_support_common::Ampacity old_ampacity = this->pp_ampacity.ampacity;
    this->pp_ampacity.ampacity = types::board_support_common::Ampacity::None;

    try {
        // this could raise a std::runtime_error
        this->pp_ampacity.ampacity = this->mod->controller.get_ampacity();

        if (old_ampacity != this->pp_ampacity.ampacity) {
            EVLOG_info << "Read PP ampacity value: " << this->pp_ampacity.ampacity;
        }

        if (this->pp_fault_reported)
            this->clear_error("evse_board_support/MREC23ProximityFault");

        // reset possible set flag since we successfully read a valid value
        this->pp_fault_reported = false;
    } catch (std::runtime_error& e) {
        EVLOG_error << e.what();

        // publish a ProximityFault
        Everest::error::Error error_object = this->error_factory->create_error(
            "evse_board_support/MREC23ProximityFault", "", e.what(), Everest::error::Severity::High);
        this->raise_error(error_object);

        // remember that we just reported the fault
        this->pp_fault_reported = true;
    }

    return this->pp_ampacity;
}

void evse_board_supportImpl::handle_ac_set_overcurrent_limit_A(double& value) {
    // your code for cmd ac_set_overcurrent_limit_A goes here
    (void)value;
}

} // namespace evse_board_support
} // namespace module
