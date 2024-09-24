// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include <chrono>
#include <iomanip>
#include <stdexcept>
#include <generated/types/cb_board_support.hpp>
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
    // configure hardware capabilities
    this->hw_capabilities.max_current_A_import = 63;
    this->hw_capabilities.min_current_A_import = 0;
    this->hw_capabilities.max_phase_count_import = 3;
    this->hw_capabilities.min_phase_count_import = 1;
    this->hw_capabilities.max_current_A_export = 63;
    this->hw_capabilities.min_current_A_export = 0;
    this->hw_capabilities.max_phase_count_export = 3;
    this->hw_capabilities.min_phase_count_export = 1;
    this->hw_capabilities.supports_changing_phases_during_charging = false;
    this->hw_capabilities.connector_type =
        types::evse_board_support::string_to_connector_type(this->mod->config.connector_type);

    // register our callback handlers
    this->mod->controller.signal_pp_state_change.connect(&evse_board_supportImpl::pp_observation_worker, this);
    this->mod->controller.signal_cp_state_change.connect(&evse_board_supportImpl::cp_observation_worker, this);
}

void evse_board_supportImpl::ready() {
    // the BSP must publish this variable at least once during start up
    this->publish_capabilities(this->hw_capabilities);
    // here too, hard-coded 3 phases for the moment
    this->publish_ac_nr_of_phases_available(3);
}

void evse_board_supportImpl::update_cp_state_internally(types::cb_board_support::CPState state,
                                                        const struct cp_state_signal_side& negative_side,
                                                        const struct cp_state_signal_side& positive_side) {
    if (this->cp_current_state == state)
        return;

    EVLOG_info << "CP state change from " << this->cp_current_state << " to " << state << ", "
               << "U_CP+: " << positive_side.voltage << " mV, "
               << "U_CP-: " << negative_side.voltage << " mV";
    this->cp_current_state = state;
}

void evse_board_supportImpl::set_duty_cycle(double value) {
    do {
        try {
            this->mod->controller.cp_set_duty_cycle(value);
            break;
        } catch (std::system_error& e) {
            EVLOG_error << e.what();

            // retry after a short period of waiting
            std::this_thread::sleep_for(this->mod->controller.get_recovery_delay_ms());
        }
    } while (true);
}

void evse_board_supportImpl::handle_enable(bool& value) {
    // generate state A or state F
    double new_duty_cycle = value ? 100.0 : 0.0;
    // pause CP observation to avoid race condition between this thread and the CP observation thread
    this->disable_cp_observation();
    EVLOG_info << "handle_enable: Setting new duty cycle of " << std::fixed << std::setprecision(2) << new_duty_cycle
               << "%";
    this->set_duty_cycle(new_duty_cycle);
    this->enable_cp_observation();
}

void evse_board_supportImpl::handle_pwm_on(double& value) {
    // pause CP observation to avoid race condition between this thread and the CP observation thread
    this->disable_cp_observation();
    EVLOG_info << "handle_pwm_on: Setting new duty cycle of " << std::fixed << std::setprecision(2) << value << "%";
    this->set_duty_cycle(value);
    this->enable_cp_observation();
}

void evse_board_supportImpl::handle_pwm_off() {
    // generate state A
    double new_duty_cycle = 100.0;
    // pause CP observation to avoid race condition between this thread and the CP observation thread
    this->disable_cp_observation();
    EVLOG_info << "handle_pwm_off: Setting new duty cycle of " << std::fixed << std::setprecision(2) << new_duty_cycle
               << "%";
    this->set_duty_cycle(new_duty_cycle);
    this->enable_cp_observation();
}

void evse_board_supportImpl::handle_pwm_F() {
    // pause CP observation to avoid race condition between this thread and the CP observation thread
    this->disable_cp_observation();
    EVLOG_info << "handle_pwm_F: Generating CP state F";
    this->set_duty_cycle(0.0);
    this->enable_cp_observation();
}

void evse_board_supportImpl::handle_allow_power_on(types::evse_board_support::PowerOnOff& value) {

    if (value.allow_power_on && this->cp_current_state == types::cb_board_support::CPState::PilotFault) {
        EVLOG_warning << "Power on rejected due to detected pilot fault.";
        return;
    }

    // TODO check whether an emergency flag is set by the safety controller which prevents to power on

    do {
        try {
            this->mod->controller.set_allow_power_on(value.allow_power_on);

            types::board_support_common::Event tmp_event = value.allow_power_on
                                                               ? types::board_support_common::Event::PowerOn
                                                               : types::board_support_common::Event::PowerOff;
            types::board_support_common::BspEvent tmp {tmp_event};
            this->publish_event(tmp);

            break;
        } catch (std::system_error& e) {
            EVLOG_error << e.what();

            // retry after a short period of waiting
            std::this_thread::sleep_for(this->mod->controller.get_recovery_delay_ms());
        }
    } while (true);
}

void evse_board_supportImpl::handle_ac_switch_three_phases_while_charging(bool& value) {
    // your code for cmd ac_switch_three_phases_while_charging goes here
}

void evse_board_supportImpl::handle_evse_replug(int& value) {
    // your code for cmd evse_replug goes here
}

types::board_support_common::ProximityPilot evse_board_supportImpl::handle_ac_read_pp_ampacity() {
    // acquire lock to guard against possible background changes done by the observation thread
    std::lock_guard<std::mutex> lock(this->pp_observation_lock);

    // pre-init to None
    this->pp_ampacity.ampacity = types::board_support_common::Ampacity::None;

    try {
        // read current ampacity from hardware
        int voltage = 0;
        this->pp_ampacity.ampacity = this->mod->controller.get_ampacity(voltage);

        EVLOG_info << "Read PP ampacity value: " << this->pp_ampacity.ampacity << " (U_PP: " << voltage << " mV)";

        if (this->pp_fault_reported)
            this->clear_error("evse_board_support/MREC23ProximityFault");

        // reset possible set flag since we successfully read a valid value
        this->pp_fault_reported = false;
    } catch (std::underflow_error& e) {
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

void evse_board_supportImpl::pp_observation_worker() {

    //EVLOG_debug << "Proximity Pilot Observation Callback called";

    // acquire lock, wait for it eventually
    this->pp_observation_lock.lock();

    try {
        // saved previous value
        types::board_support_common::Ampacity prev_value(this->pp_ampacity.ampacity);

        // read new/actual ampacity from hardware
        int voltage = 0;
        this->pp_ampacity.ampacity = this->mod->controller.get_ampacity(voltage);

        if (this->pp_ampacity.ampacity != prev_value) {

            if (this->pp_ampacity.ampacity == types::board_support_common::Ampacity::None) {
                EVLOG_info << "PP noticed plug removal from socket (U_PP: " << voltage << " mV)";
            } else {
                EVLOG_info << "PP ampacity change from " << prev_value << " to " << this->pp_ampacity.ampacity
                           << " (U_PP: " << voltage << " mV)";
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

            if (this->pp_ampacity.ampacity != types::board_support_common::Ampacity::None &&
                this->pp_fault_reported) {
                // clear a ProximityFault error on PP state change to a valid value but only if it exists
                this->clear_error("evse_board_support/MREC23ProximityFault");
                this->pp_fault_reported = false;
            }
        }
    } catch (std::underflow_error& e) {
        if (!this->pp_fault_reported) {
            EVLOG_error << e.what();

            // publish a ProximityFault
            Everest::error::Error error_object = this->error_factory->create_error(
                "evse_board_support/MREC23ProximityFault", "", e.what(), Everest::error::Severity::High);
            this->raise_error(error_object);

            this->pp_fault_reported = true;
        }
    }

    this->pp_observation_lock.unlock();

    //EVLOG_debug << "Proximity Pilot Observation Callback finished";
}

void evse_board_supportImpl::handle_ac_set_overcurrent_limit_A(double& value) {
    // your code for cmd ac_set_overcurrent_limit_A goes here
}

void evse_board_supportImpl::disable_cp_observation() {
    if (this->cp_observation_enabled) {
        EVLOG_debug << "Disabling CP observation";
        this->cp_observation_lock.lock();
        this->cp_observation_enabled = false;
    } else {
        EVLOG_debug << "Disabling CP observation (suppressed)";
    }
}

void evse_board_supportImpl::enable_cp_observation() {
    if (!this->cp_observation_enabled) {
        EVLOG_debug << "Enabling CP observation";
        this->cp_observation_enabled = true;
        this->cp_observation_lock.unlock();
    } else {
        EVLOG_debug << "Enabling CP observation (suppressed)";
    }
}

bool evse_board_supportImpl::cp_state_changed(struct cp_state_signal_side& signal_side) {
    bool rv {false};

    // CP state is only detected if the new state is different from the previous one (first condition).
    // Additionally, to filter simple disturbances, a new state must be detected twice before notifying it (second
    // condition). For that, we need at least two CP state measurements (third condition)

    if (signal_side.previous_state != signal_side.measured_state &&
        signal_side.current_state == signal_side.measured_state) {

        // update the previous state
        signal_side.previous_state = signal_side.current_state;
        rv = true;

    } else if (signal_side.measured_state == signal_side.previous_state &&
               signal_side.measured_state != signal_side.current_state) {
        EVLOG_warning << "CP state change from " << signal_side.previous_state << " to " << signal_side.current_state
                      << " suppressed";
    }

    signal_side.current_state = signal_side.measured_state;

    return rv;
}

void evse_board_supportImpl::cp_observation_worker() {
    bool duty_cycle_changed {false};
    bool cp_state_changed {false};

    if (!this->cp_observation_enabled) {
        //EVLOG_debug << "Control Pilot Observation Callback suppressed (not enabled <yet>)";
        return;
    }

    //EVLOG_debug << "Control Pilot Observation Callback called";

    // acquire measurement lock for this loop round, wait for it eventually
    std::lock_guard<std::mutex> lock(this->cp_observation_lock);

    // do the actual measurement
    this->mod->controller.cp_get_values(this->cp_positive_side.voltage, this->cp_negative_side.voltage);

    // positive signal side: map to CP state and check for changes
    this->cp_positive_side.measured_state =
        this->mod->controller.cp_voltage_to_state(this->cp_positive_side.voltage, this->cp_positive_side.current_state);
    cp_state_changed |= this->cp_state_changed(this->cp_positive_side);

    // negative signal side: map to CP state and check for changes
    this->cp_negative_side.measured_state =
        this->mod->controller.cp_voltage_to_state(this->cp_negative_side.voltage, this->cp_negative_side.current_state);
    cp_state_changed |= this->cp_state_changed(this->cp_negative_side);

    // at this point, the current_state member was already updated by the cp_state_changed methods

    // check whether we see a change of the duty cycle
    duty_cycle_changed = this->previous_duty_cycle != this->mod->controller.cp_get_duty_cycle();
    this->previous_duty_cycle = this->mod->controller.cp_get_duty_cycle();

    // if there was actually a change -> tell it to upper layers
    if (cp_state_changed || duty_cycle_changed) {
        // A diode fault is detected if the nominal duty cycle is set and the negative side of
        // the CP is not in state F (-12V)
        if (this->mod->controller.cp_is_nominal_duty_cycle() &&
            this->cp_negative_side.current_state != types::cb_board_support::CPState::F) {
            if (!this->diode_fault_reported) {
                this->diode_fault_reported = true;
                this->update_cp_state_internally(types::cb_board_support::CPState::PilotFault,
                                                 this->cp_negative_side, this->cp_positive_side);
                Everest::error::Error error_object = this->error_factory->create_error(
                    "evse_board_support/DiodeFault", "", "Diode fault detected.", Everest::error::Severity::High);
                this->raise_error(error_object);
            }
            return;
        }
    }

    if (cp_state_changed) {
        // in case we drive state F, then we cannot trust the peak detectors
        if (this->mod->controller.cp_get_duty_cycle() == 0.0) {
            types::board_support_common::BspEvent tmp {types::board_support_common::Event::F};
            this->publish_event(tmp);
            this->update_cp_state_internally(types::cb_board_support::CPState::F, cp_negative_side, cp_positive_side);
            return;
        }
        // normal CP state change
        if (this->cp_positive_side.current_state != this->cp_current_state) {
            // in case we see a pilot fault, we need to inform the upper layer
            if (this->cp_positive_side.current_state == types::cb_board_support::CPState::PilotFault ||
                    this->cp_negative_side.current_state == types::cb_board_support::CPState::PilotFault) {
                if (this->pilot_fault_reported == false) {
                    Everest::error::Error error_object =
                        this->error_factory->create_error("evse_board_support/MREC14PilotFault", "",
                                                          "Pilot fault detected.", Everest::error::Severity::High);
                    this->raise_error(error_object);
                    this->pilot_fault_reported = true;
                    this->update_cp_state_internally(types::cb_board_support::CPState::PilotFault,
                                                     this->cp_negative_side, this->cp_positive_side);
                }
                return;
            }
            // Before checking for ventilation error, we need to clear other errors if they were reported before
            if (this->diode_fault_reported) {
                this->clear_error("evse_board_support/DiodeFault");
                this->diode_fault_reported = false;
            }
            if (this->pilot_fault_reported) {
                this->clear_error("evse_board_support/MREC14PilotFault");
                this->pilot_fault_reported = false;
            }
            // check if a ventilation error has occurred
            if (this->cp_positive_side.current_state == types::cb_board_support::CPState::D) {
                // in case we see a ventilation request, although we do not support it,
                // we need to inform the upper layer
                Everest::error::Error error_object = this->error_factory->create_error(
                    "evse_board_support/VentilationNotAvailable", "", "Ventilation fault detected.",
                    Everest::error::Severity::High);
                this->raise_error(error_object);
                this->ventilation_fault_reported = true;
                this->publish_event({types::board_support_common::Event::D});
                this->update_cp_state_internally(this->cp_positive_side.current_state, this->cp_negative_side, this->cp_positive_side);
                return;
            }
            if (this->ventilation_fault_reported) {
                this->clear_error("evse_board_support/VentilationNotAvailable");
                this->ventilation_fault_reported = false;
            }
            try {
                types::board_support_common::BspEvent tmp = cpstate_to_bspevent(this->cp_positive_side.current_state);
                this->publish_event(tmp);
                this->update_cp_state_internally(this->cp_positive_side.current_state, this->cp_negative_side, this->cp_positive_side);
            } catch (std::runtime_error& e) {
                // Should never happen, when all invalid states are handled correctly
                EVLOG_warning << e.what();
            }
        }
    }

    //EVLOG_debug << "Control Pilot Observation Callback finished";
}

} // namespace evse_board_support
} // namespace module
