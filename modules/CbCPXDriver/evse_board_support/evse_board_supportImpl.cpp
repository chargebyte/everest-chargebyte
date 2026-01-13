// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "evse_board_supportImpl.hpp"

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

    this->hw_capabilities.supports_changing_phases_during_charging = false;
    this->hw_capabilities.max_phase_count_import = 3;
    this->hw_capabilities.min_phase_count_import = 3;
    this->hw_capabilities.max_phase_count_export = 3;
    this->hw_capabilities.min_phase_count_export = 3;

    this->hw_capabilities.connector_type =
        types::evse_board_support::string_to_connector_type(this->mod->config.connector_type);

    // register our callback handlers

    this->mod->controller->on_pp_change.connect([this](uint8_t new_pp_state) {
        std::scoped_lock lock(this->pp_mutex);

        try {
            // saved previous value
            const types::board_support_common::Ampacity prev_value(this->pp_ampacity.ampacity);

            this->pp_ampacity.ampacity = this->mod->controller->pp_state_to_ampacity(new_pp_state);

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
        } catch (const std::runtime_error& e) {
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

    this->mod->controller->on_cp_change.connect([this](uint8_t current_cp_state) {
        std::scoped_lock lock(this->cp_mutex);

        types::cb_board_support::CPState tmp_current_cp_state =
            static_cast<types::cb_board_support::CPState>(current_cp_state);

        // FIXME REVISIT
        // safety controller can report CP state unknown (which maps to our PowerOn here) in case
        // of e.g short-circuit (should be clarified with safety fw development whether better report E)
        if (tmp_current_cp_state == types::cb_board_support::CPState::PowerOn)
            return;

        // FIXME REVISIT special case: during power on filter out transient state E
        if (this->cp_current_state == types::cb_board_support::CPState::PowerOn &&
            tmp_current_cp_state == types::cb_board_support::CPState::E) {
            EVLOG_debug << "CP state change from " << this->cp_current_state << " to " << tmp_current_cp_state << ", "
                        << "PWM: " << std::fixed << std::setprecision(1)
                        << (this->mod->controller->get_duty_cycle() / 10.0) << "%" << " [filtered]";
            return;
        }

        EVLOG_info << "CP state change from " << this->cp_current_state << " to " << tmp_current_cp_state << ", "
                   << "PWM: " << std::fixed << std::setprecision(1) << (this->mod->controller->get_duty_cycle() / 10.0)
                   << "%";

        // we can determine this directly from the currently seen value
        this->cp_errors.pilot_fault.is_active = tmp_current_cp_state == types::cb_board_support::CPState::PilotFault;

        CPUtils::process_everest_errors(*this, this->cp_errors.errors);

        this->cp_current_state = tmp_current_cp_state;

        if (tmp_current_cp_state == types::cb_board_support::CPState::PilotFault)
            return;

        try {
            const types::board_support_common::BspEvent tmp = cpstate_to_bspevent(tmp_current_cp_state);
            this->publish_event(tmp);
        } catch (const std::runtime_error& e) {
            // should never happen, when all invalid states are handled correctly
            EVLOG_warning << e.what();
        }
    });

    this->mod->controller->on_cp_error.connect([this]() {
        std::scoped_lock lock(this->cp_mutex);

        // we can determine this directly from the currently seen values
        this->cp_errors.diode_fault.is_active = this->mod->controller->get_cs_diode_fault();
        this->cp_errors.cp_short_fault.is_active = this->mod->controller->get_cs_short_circuit();

        CPUtils::process_everest_errors(*this, this->cp_errors.errors);

        // we only need to remember the PilotFault here, reset is done in `on_cp_change`
        if (this->cp_errors.diode_fault.is_active || this->cp_errors.cp_short_fault.is_active) {
            this->cp_current_state = types::cb_board_support::CPState::PilotFault;
        }
    });

    this->mod->controller->on_contactor_error.connect(
        [this](const std::string& source, bool desired_state, types::cb_board_support::ContactorState actual_state) {
            raise_contactor_error(source, desired_state, actual_state);
        });

    this->mod->controller->on_estop.connect([this](const unsigned int& estop, const bool& active) {
        if (active)
            EVLOG_warning << "Emergency Stop " << estop << " TRIPPED";
        else
            EVLOG_info << "Emergency Stop " << estop << " released";
    });

    this->mod->controller->on_cpx_timeout.connect([this](bool timeout) {
        const std::string error_type = "evse_board_support/VendorError";
        if (timeout == true) {
            // signal State A so that EvseManager cancels session and starts
            // fresh once CPX is back
            this->cp_current_state = types::cb_board_support::CPState::A;
            try {
                const types::board_support_common::BspEvent tmp = cpstate_to_bspevent(this->cp_current_state);
                this->publish_event(tmp);
            } catch (const std::runtime_error& e) {
                // should never happen, when all invalid states are handled correctly
                EVLOG_warning << e.what();
            }

            // raise VendorError because of timeout
            std::ostringstream errmsg;
            errmsg << "CPX timeout registered";

            EVLOG_warning << errmsg.str() << ", raising VendorError.";

            Everest::error::Error error_object =
                this->error_factory->create_error(error_type, "", errmsg.str(), Everest::error::Severity::High);
            this->raise_error(error_object);
        } else {
            // clear error after CPX is back after timeout
            this->clear_error(error_type);
        }
    });
}

void evse_board_supportImpl::ready() {
    // the BSP must publish this variable at least once during start up
    this->publish_capabilities(this->hw_capabilities);

    // the PP ampacity should be published during start up
    this->pp_ampacity.ampacity = this->mod->controller->get_ampacity();
    this->publish_ac_pp_ampacity(this->pp_ampacity);
}

void evse_board_supportImpl::handle_enable(bool& value) {
    try {
        // enable CPX communication
        if (value)
            this->mod->controller->enable();

        // generate state A or state F
        const unsigned int new_duty_cycle = value ? 1000 : 0;

        EVLOG_info << "handle_enable: Setting new duty cycle of " << std::fixed << std::setprecision(1)
                   << (new_duty_cycle / 10.0) << "%";
        this->mod->controller->set_duty_cycle(new_duty_cycle);

        this->is_enabled = value;
    } catch (const std::exception& e) {
        EVLOG_error << e.what();
    }
}

void evse_board_supportImpl::handle_pwm_on(double& value) {
    try {
        const unsigned int new_duty_cycle = static_cast<unsigned int>(value * 10.0);

        EVLOG_info << "handle_pwm_on: Setting new duty cycle of " << std::fixed << std::setprecision(1)
                   << (new_duty_cycle / 10.0) << "%";
        this->mod->controller->set_duty_cycle(new_duty_cycle);
    } catch (const std::exception& e) {
        EVLOG_error << e.what();
    }
}

void evse_board_supportImpl::handle_pwm_off() {
    // in case safety controller was in emergency state, we have to reset it
    // with a disable -> enable toggle
    if (this->mod->controller->is_emergency()) {
        EVLOG_info << "handle_pwm_off: recovering after safety state";

        // FIXME: figure out viable solution to handle emergency and then implement
    }

    try {
        // generate state A
        const unsigned int new_duty_cycle = 1000;

        EVLOG_info << "handle_pwm_off: Setting new duty cycle of " << std::fixed << std::setprecision(1)
                   << (new_duty_cycle / 10.0) << "%";
        this->mod->controller->set_duty_cycle(new_duty_cycle);
    } catch (const std::exception& e) {
        EVLOG_error << e.what();
    }
}

void evse_board_supportImpl::handle_pwm_F() {
    try {
        // generate state F
        const unsigned int new_duty_cycle = 0;

        EVLOG_info << "handle_pwm_F: Generating CP state F";

        this->mod->controller->set_duty_cycle(new_duty_cycle);
    } catch (const std::exception& e) {
        EVLOG_error << e.what();
    }
}

void evse_board_supportImpl::handle_allow_power_on(types::evse_board_support::PowerOnOff& value) {
    // this method is called very often, even the contactor state is already matching the desired one
    // so let's use this as helper to control the log noise a little bit and whether we actually
    // need to report a BSP event
    const bool contactor_state = this->mod->controller->get_contactor_state();

    const bool state_change = value.allow_power_on != contactor_state;

    if (value.allow_power_on && this->cp_current_state == types::cb_board_support::CPState::PilotFault) {
        EVLOG_warning << "Power on rejected: pilot fault detected.";
        return;
    }

    if (value.allow_power_on && this->mod->controller->is_emergency()) {
        EVLOG_warning << "Power on rejected: emergency state active.";
        return;
    }

    if (value.allow_power_on && this->contactor_fault_reported) {
        EVLOG_warning << "Power on rejected: contactor fault present.";
        return;
    }

    std::ostringstream ss;
    ss << "handle_allow_power_on: request to " << (value.allow_power_on ? "CLOSE" : "OPEN") << " the contactor";

    if (state_change)
        EVLOG_info << ss.str();
    else
        EVLOG_debug << ss.str();

    // exit early if we don't actually change the state
    if (!state_change) {
        EVLOG_info << "Current (unchanged) contactor state: " << (contactor_state ? "CLOSED" : "OPEN");
        return;
    }

    const int code_switch_state = this->mod->controller->switch_state(value.allow_power_on);

    if (code_switch_state == 0) {
        EVLOG_info << "Current contactor state: " << (contactor_state ? "OPEN" : "CLOSED");

        // publish PowerOn or PowerOff event
        const types::board_support_common::Event tmp_event = value.allow_power_on
                                                                 ? types::board_support_common::Event::PowerOn
                                                                 : types::board_support_common::Event::PowerOff;
        types::board_support_common::BspEvent tmp {tmp_event};
        this->publish_event(tmp);
    } else {
        const types::cb_board_support::ContactorState actual_state =
            !(value.allow_power_on) ? types::cb_board_support::ContactorState::Closed
                                    : types::cb_board_support::ContactorState::Open;
        raise_contactor_error("Contactor " + std::to_string(code_switch_state), value.allow_power_on, actual_state);
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

void evse_board_supportImpl::handle_ac_set_overcurrent_limit_A(double& value) {
    // your code for cmd ac_set_overcurrent_limit_A goes here
    (void)value;
}

void evse_board_supportImpl::raise_contactor_error(const std::string& source, bool desired_state,
                                                   types::cb_board_support::ContactorState actual_state) {
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
}

} // namespace evse_board_support
} // namespace module
