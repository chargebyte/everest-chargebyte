// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include <chrono>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <generated/types/cb_board_support.hpp>
#include <CbTarragonContactorControl.hpp>
#include <CbTarragonContactorControlSimple.hpp>
#include <CbTarragonContactorControlSerial.hpp>
#include <CbTarragonContactorControlMutual.hpp>

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
    // just for clarity
    this->termination_requested = false;
    this->pp_fault_reported = false;

    // configure hardware capabilities: use user-configurable settings for flexibility
    // but use the same value for import and export - there seems to be no reason for AC
    // that these values differ for import and export
    this->hw_capabilities.min_current_A_import = this->mod->config.min_current_A;
    this->hw_capabilities.max_current_A_import = this->mod->config.max_current_A;
    this->hw_capabilities.min_current_A_export = this->mod->config.min_current_A;
    this->hw_capabilities.max_current_A_export = this->mod->config.max_current_A;

    // check whether the configuration allows to enable phase-count switching support:
    // - 'switch_3ph1ph_wiring' must not be 'none'
    bool support_3ph1ph = this->mod->config.switch_3ph1ph_wiring != "none";

    this->hw_capabilities.supports_changing_phases_during_charging = support_3ph1ph;
    this->hw_capabilities.max_phase_count_import = 3;
    this->hw_capabilities.min_phase_count_import = support_3ph1ph ? 1 : 3;
    this->hw_capabilities.max_phase_count_export = 3;
    this->hw_capabilities.min_phase_count_export = support_3ph1ph ? 1 : 3;

    this->hw_capabilities.connector_type =
        types::evse_board_support::string_to_connector_type(this->mod->config.connector_type);

    // Control Pilot state observation
    this->cp_controller =
        CbTarragonCP(this->mod->config.cp_pos_peak_adc_device, this->mod->config.cp_pos_peak_adc_channel,
                     this->mod->config.cp_rst_pos_peak_gpio_line_name, this->mod->config.cp_neg_peak_adc_device,
                     this->mod->config.cp_neg_peak_adc_channel, this->mod->config.cp_rst_neg_peak_gpio_line_name);

    // Control Pilot PWM generation
    this->pwm_controller = CbTarragonPWM(this->mod->config.cp_pwm_device, this->mod->config.cp_pwmchannel,
                                         this->mod->config.cp_invert_gpio_line_name);

    // preset with PowerOn for nice logging
    this->cp_current_state = types::cb_board_support::CPState::PowerOn;

    // start cp-observation-worker thread
    this->cp_observation_thread = std::thread(&evse_board_supportImpl::cp_observation_worker, this);

    // Proximity Pilot state observation
    this->pp_controller = CbTarragonPP(this->mod->config.pp_adc_device, this->mod->config.pp_adc_channel);

    // connect contactor controller signals
    this->mod->contactor_controller->on_error.connect(
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
    this->mod->contactor_controller->on_unexpected_change.connect(
        [&](const std::string& source, types::cb_board_support::ContactorState seen_state) {
            // A spurious change on the feedback line should not happen and is unexpected.
            // This fault is critical as it might be a sign of a hardware issue, therefore
            // the fault will not be cleared to prevent further damage.
            std::ostringstream errmsg;
            errmsg << "Spurious state '" << seen_state << "' on feedback signal of '" << source << "' detected";

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
}

void evse_board_supportImpl::ready() {
    std::ostringstream errmsg;

    // The BSP must publish this variable at least once during start up.
    this->publish_capabilities(this->hw_capabilities);

    // let's pre-init the error message: the `is_inconsistent_state` method appends to the stream
    // in case of error, otherwise the stream is left untouched
    errmsg << "Initial contactor feedback check failed for ";

    // Let's check the current contactor state and raise a contactor fault in case there is anything unexpected
    if (this->mod->contactor_controller->is_inconsistent_state(errmsg)) {
        // raise a contactor fault if it was not done before
        if (!this->contactor_fault_reported.exchange(true)) {
            EVLOG_error << errmsg.str()
                        << ", raising MREC17EVSEContactorFault. Please double-check your configuration.";

            Everest::error::Error error_object = this->error_factory->create_error(
                "evse_board_support/MREC17EVSEContactorFault", "", errmsg.str(), Everest::error::Severity::High);
            this->raise_error(error_object);
        } else {
            EVLOG_error << errmsg.str() << ". Please double-check your configuration.";
        }
    }
}

void evse_board_supportImpl::update_cp_state_internally(types::cb_board_support::CPState state,
                                                        const CPUtils::cp_state_signal_side& negative_side,
                                                        const CPUtils::cp_state_signal_side& positive_side) {
    std::stringstream duty_msg;

    if (this->pwm_controller.is_enabled())
        duty_msg << std::fixed << std::setprecision(2) << this->pwm_controller.get_duty_cycle() << "%)";
    else
        duty_msg << "off";

    EVLOG_info << "CP state change from " << this->cp_current_state << " to " << state << ", "
               << "U_CP+: " << positive_side.voltage << " mV, "
               << "U_CP-: " << negative_side.voltage << " mV, "
               << "PWM: " << duty_msg.str();
    this->cp_current_state = state;
}

evse_board_supportImpl::~evse_board_supportImpl() {
    // request termination of our worker threads
    this->termination_requested = true;

    // if thread is active wait until it is terminated
    if (this->cp_observation_thread.joinable())
        this->cp_observation_thread.join();

    // if thread exists and is active wait until it is terminated
    if (this->pp_observation_thread.joinable())
        this->pp_observation_thread.join();
}

void evse_board_supportImpl::handle_enable(bool& value) {
    // pause CP observation to avoid race condition between this thread and the CP observation thread
    std::scoped_lock lock(this->cp_observation_lock);

    this->is_enabled = value;
    this->is_enabled_changed.notify_one();

    // generate state A or state F
    double new_duty_cycle = value ? 100.0 : 0.0;

    EVLOG_info << "handle_enable: Setting new duty cycle of " << std::fixed << std::setprecision(2) << new_duty_cycle
               << "%";
    this->pwm_controller.set_duty_cycle(new_duty_cycle);
}

void evse_board_supportImpl::handle_pwm_on(double& value) {
    // pause CP observation to avoid race condition between this thread and the CP observation thread
    std::scoped_lock lock(this->cp_observation_lock);

    std::stringstream amps_msg;
    double amps;

    // calculate the corresponding limit in Ampere, just for logging
    if (value >= 85.0)
        amps = (value - 64.0) * 2.5;
    else
        amps = value * 0.6;
    if (10.0 <= value && value <= 96.0)
        amps_msg << " (" << std::fixed << std::setprecision(1) << amps << " A)";

    EVLOG_info << "handle_pwm_on: Setting new duty cycle of " << std::fixed << std::setprecision(2) << value << "%"
               << amps_msg.str();
    this->pwm_controller.set_duty_cycle(value);
}

void evse_board_supportImpl::handle_pwm_off() {
    // pause CP observation to avoid race condition between this thread and the CP observation thread
    std::scoped_lock lock(this->cp_observation_lock);

    // generate state A
    double new_duty_cycle = 100.0;

    EVLOG_info << "handle_pwm_off: Setting new duty cycle of " << std::fixed << std::setprecision(2) << new_duty_cycle
               << "%";
    this->pwm_controller.set_duty_cycle(100.0);
}

void evse_board_supportImpl::handle_pwm_F() {
    // pause CP observation to avoid race condition between this thread and the CP observation thread
    std::scoped_lock lock(this->cp_observation_lock);

    EVLOG_info << "Generating CP state F";
    this->pwm_controller.set_duty_cycle(0.0);
}

void evse_board_supportImpl::handle_cp_state_E() {
	// pause CP observation to avoid race condition between this thread and the CP observation thread
    std::scoped_lock lock(this->cp_observation_lock);

    EVLOG_info << "Generating CP state E";
    this->pwm_controller.disable();
}

void evse_board_supportImpl::handle_allow_power_on(types::evse_board_support::PowerOnOff& value) {
    // this method is called very often, even the contactor state is already matching the desired one
    // so let's use this as helper to control the log noise a little bit and whether we actually
    // need to report a BSP event
    bool state_change = value.allow_power_on != this->mod->contactor_controller->get_state();

    if (value.allow_power_on && this->cp_current_state == types::cb_board_support::CPState::PilotFault) {
        EVLOG_warning << "Power on rejected due to detected pilot fault";
        return;
    }

    if (value.allow_power_on && this->mod->contactor_controller->is_emergency) {
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
        EVLOG_info << "Current (unchanged) state: " << *this->mod->contactor_controller;
        return;
    }

    if (value.allow_power_on) {
        auto delay = this->mod->contactor_controller->get_closing_delay_left();
        if (delay > 0ms) {
            std::chrono::duration<double> seconds = std::chrono::duration_cast<std::chrono::duration<double>>(delay);

            EVLOG_warning << "Contactor close throttling in effect: closing will be delayed by " << std::fixed
                          << std::setprecision(1) << seconds.count() << "s";
        }
    }
    if (this->mod->contactor_controller->switch_state(value.allow_power_on)) {
        EVLOG_info << "Current state: " << *this->mod->contactor_controller;

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

    this->mod->contactor_controller->switch_phase_count(value);
}

void evse_board_supportImpl::handle_evse_replug(int& value) {
    // your code for cmd evse_replug goes here
    (void)value;
}

types::board_support_common::ProximityPilot evse_board_supportImpl::handle_ac_read_pp_ampacity() {
    // acquire lock to guard against possible background changes done by the observation thread
    std::lock_guard<std::mutex> lock(this->pp_observation_lock);
    types::board_support_common::Ampacity old_ampacity = this->pp_ampacity.ampacity;

    // pre-init to None
    this->pp_ampacity.ampacity = types::board_support_common::Ampacity::None;

    // start PP observation worker thread if not yet done
    // we stop and wait at the lock by us
    if (!this->pp_observation_thread.joinable())
        this->pp_observation_thread = std::thread(&evse_board_supportImpl::pp_observation_worker, this);

    try {
        // read current ampacity from hardware
        int voltage = 0;
        this->pp_ampacity.ampacity = this->pp_controller.get_ampacity(voltage);

        if (old_ampacity != this->pp_ampacity.ampacity) {
            EVLOG_info << "Read PP ampacity value: " << this->pp_ampacity.ampacity << " (U_PP: " << voltage << " mV)";
        }

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

void evse_board_supportImpl::pp_observation_worker(void) {

    EVLOG_info << "Proximity Pilot Observation Thread started";

    while (!this->termination_requested) {
        // acquire lock, wait for it eventually
        this->pp_observation_lock.lock();

        try {
            // saved previous value
            types::board_support_common::Ampacity prev_value(this->pp_ampacity.ampacity);

            // read new/actual ampacity from hardware
            int voltage = 0;
            this->pp_ampacity.ampacity = this->pp_controller.get_ampacity(voltage);

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
                    if (!this->pp_fault_reported.exchange(true)) {
                        Everest::error::Error error_object = this->error_factory->create_error(
                            "evse_board_support/MREC23ProximityFault", "", "Plug removed from socket during charge",
                            Everest::error::Severity::High);
                        this->raise_error(error_object);
                    }
                }

                // clear a ProximityFault error on PP state change to a valid value but only if it exists
                if (this->pp_ampacity.ampacity != types::board_support_common::Ampacity::None &&
                    this->pp_fault_reported.exchange(false)) {
                    this->clear_error("evse_board_support/MREC23ProximityFault");
                }
            }
        } catch (std::underflow_error& e) {
            if (!this->pp_fault_reported.exchange(true)) {
                EVLOG_error << e.what();

                // publish a ProximityFault
                Everest::error::Error error_object = this->error_factory->create_error(
                    "evse_board_support/MREC23ProximityFault", "", e.what(), Everest::error::Severity::High);
                this->raise_error(error_object);
            }
        }

        this->pp_observation_lock.unlock();

        // break before sleeping in case of already requested termination
        if (this->termination_requested)
            break;

        // let's sleep for a "randomly" selected time: 500ms should be a good trade-off between
        // CPU usage and fast recognition of any kind of trouble with the PP line
        std::this_thread::sleep_for(500ms);
    }

    EVLOG_info << "Proximity Pilot Observation Thread terminated";
}

void evse_board_supportImpl::handle_ac_set_overcurrent_limit_A(double& value) {
    // your code for cmd ac_set_overcurrent_limit_A goes here
    (void)value;
}

types::cb_board_support::CPState
evse_board_supportImpl::determine_cp_state(const CPUtils::cp_state_signal_side& cp_state_positive_side,
                                           const CPUtils::cp_state_signal_side& cp_state_negative_side,
                                           const double& duty_cycle, bool& is_cp_error) {
    // In case the PWM is disabled, we cannot trust the peak detectors
    if (!this->pwm_controller.is_enabled()) {
        return types::cb_board_support::CPState::E;
    }

    // In case we drive state F (0% PWM), then we cannot trust the peak detectors
    if (duty_cycle == 0.0) {
        return types::cb_board_support::CPState::F;
    }

    // Check for CP errors
    types::cb_board_support::CPState current_cp_state = cp_state_positive_side.measured_state_t1;
    if (cp_state_positive_side.measured_state_t1 == types::cb_board_support::CPState::PilotFault ||
        cp_state_negative_side.measured_state_t1 == types::cb_board_support::CPState::PilotFault) {
        current_cp_state = types::cb_board_support::CPState::PilotFault;
    }
    is_cp_error = CPUtils::check_for_cp_errors(this->cp_errors, current_cp_state, this->pwm_controller.get_duty_cycle(),
                                               cp_state_negative_side.voltage, cp_state_positive_side.voltage);
    if (is_cp_error) {
        return current_cp_state;
    }

    return cp_state_positive_side.detected_state;
}

void evse_board_supportImpl::cp_observation_worker(void) {
    double previous_duty_cycle {100.0};
    // both sides of the CP level
    CPUtils::cp_state_signal_side positive_side {types::cb_board_support::CPState::PilotFault,
                                                 types::cb_board_support::CPState::PilotFault,
                                                 types::cb_board_support::CPState::PilotFault, 0};
    CPUtils::cp_state_signal_side negative_side {types::cb_board_support::CPState::PilotFault,
                                                 types::cb_board_support::CPState::PilotFault,
                                                 types::cb_board_support::CPState::PilotFault, 0};

    EVLOG_info << "Control Pilot Observation Thread started";

    while (!this->termination_requested) {
        bool duty_cycle_changed {false};
        bool cp_state_changed {false};

        // acquire measurement lock for this loop round, wait for it eventually
        std::unique_lock<std::mutex> lock(this->cp_observation_lock);

        // when this EVSE is not (yet) enabled, we are not allowed to publish BSP events
        // so we must wait until `is_enabled` is true
        if (!this->is_enabled)
            this->is_enabled_changed.wait(lock, [this]() {
                // we want to wait until we are enabled (again), or we have to
                // return in case termination was requested while waiting
                return this->is_enabled or this->termination_requested;
            });

        // exit the loop if termination was requested
        if (this->termination_requested)
            break;

        if (!this->pwm_controller.is_enabled()) {
            std::this_thread::sleep_for(2ms);
            continue;
        }

        // do the actual measurement
        this->cp_controller.get_values(positive_side.voltage, negative_side.voltage);

        // positive signal side: map to CP state and check for changes
        types::cb_board_support::CPState measured_cp_state;
        measured_cp_state = CPUtils::voltage_to_state(positive_side.voltage, positive_side.measured_state_t1);
        cp_state_changed |= CPUtils::check_for_cp_state_changes(positive_side, measured_cp_state);

        // negative signal side: map to CP state and check for changes
        measured_cp_state = CPUtils::voltage_to_state(negative_side.voltage, negative_side.measured_state_t1);
        cp_state_changed |= CPUtils::check_for_cp_state_changes(negative_side, measured_cp_state);

        // at this point, the current_state member was already updated by the check_for_cp_state_changes methods

        // check whether we see a change of the duty cycle
        duty_cycle_changed = previous_duty_cycle != this->pwm_controller.get_duty_cycle();
        previous_duty_cycle = this->pwm_controller.get_duty_cycle();

        // If nothing has changed, start a new measurement
        if (duty_cycle_changed == false && cp_state_changed == false) {
            continue;
        }

        // Determine current CP state based on positive, negative side and duty cycle
        bool is_cp_error {false};
        types::cb_board_support::CPState current_cp_state =
            determine_cp_state(positive_side, negative_side, this->pwm_controller.get_duty_cycle(), is_cp_error);

        // Process all EVerest CP errors
        CPUtils::process_everest_errors(*this, this->cp_errors.errors);

        // Normal CP state change
        try {
            if (current_cp_state != types::cb_board_support::CPState::PilotFault) {
                const types::board_support_common::BspEvent tmp = cpstate_to_bspevent(current_cp_state);
                this->publish_event(tmp);
            }
        } catch (std::runtime_error& e) {
            // Should never happen, when all invalid states are handled correctly
            EVLOG_warning << e.what();
        }
        if ((current_cp_state != this->cp_current_state) || is_cp_error) {
            this->update_cp_state_internally(current_cp_state, negative_side, positive_side);
        }

        // clear a possible ProximityFault error on transition to state A
        if (current_cp_state == types::cb_board_support::CPState::A && this->pp_fault_reported.exchange(false)) {
            this->clear_error("evse_board_support/MREC23ProximityFault");
        }
    }

    EVLOG_info << "Control Pilot Observation Thread terminated";
}

} // namespace evse_board_support
} // namespace module
