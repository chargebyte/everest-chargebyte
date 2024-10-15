// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

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
    // just for clarity
    this->termination_requested = false;
    this->pp_fault_reported = false;
    this->pilot_fault_reported = false;
    this->diode_fault_reported = false;
    this->ventilation_fault_reported = false;

    // Configure hardware capabilities
    this->hw_capabilities.max_current_A_import = 32;
    this->hw_capabilities.min_current_A_import = 0;
    this->hw_capabilities.max_phase_count_import = 3;
    this->hw_capabilities.min_phase_count_import = 1;
    this->hw_capabilities.max_current_A_export = 32;
    this->hw_capabilities.min_current_A_export = 0;
    this->hw_capabilities.max_phase_count_export = 3;
    this->hw_capabilities.min_phase_count_export = 1;
    this->hw_capabilities.supports_changing_phases_during_charging = false;
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

    // acquire the lock so that the CP observation does not start immediately
    this->cp_observation_lock.lock();
    this->cp_observation_enabled = false;

    // preset with PowerOn for nice logging
    this->cp_current_state = types::cb_board_support::CPState::PowerOn;

    // start cp-observation-worker thread
    this->cp_observation_thread = std::thread(&evse_board_supportImpl::cp_observation_worker, this);

    // Proximity Pilot state observation
    this->pp_controller = CbTarragonPP(this->mod->config.pp_adc_device, this->mod->config.pp_adc_channel);

    // Relay and contactor handling
    this->contactor_controller = CbTarragonContactorControl(
        this->mod->config.relay_1_name, this->mod->config.relay_1_actuator_gpio_line_name,
        this->mod->config.relay_1_feedback_gpio_line_name, this->mod->config.contactor_1_feedback_type,
        this->mod->config.relay_2_name, this->mod->config.relay_2_actuator_gpio_line_name,
        this->mod->config.relay_2_feedback_gpio_line_name, this->mod->config.contactor_2_feedback_type);

    // set the contactor handling related flags and timeout
    this->contactor_feedback_timeout = 200ms;
    this->allow_power_on = false;
    this->last_allow_power_on = false;

    // start contactor handling thread
    this->contactor_handling_thread = std::thread(&evse_board_supportImpl::contactor_handling_worker, this);
}

void evse_board_supportImpl::ready() {
    // The BSP must publish this variable at least once during start up.
    this->publish_capabilities(this->hw_capabilities);
}

void evse_board_supportImpl::set_emergency_state(bool is_emergency) {
    evse_board_supportImpl::is_emergency = is_emergency;
    EVLOG_info << (is_emergency ? "emergency occurred" : "emergency released");
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

evse_board_supportImpl::~evse_board_supportImpl() {
    // request termination of our worker threads
    this->termination_requested = true;

    // ensure that thread is not waiting for the lock
    if (!this->cp_observation_enabled)
        this->cp_observation_lock.unlock();

    // if thread is active wait until it is terminated
    if (this->cp_observation_thread.joinable())
        this->cp_observation_thread.join();

    // if thread exists and is active wait until it is terminated
    if (this->pp_observation_thread.joinable())
        this->pp_observation_thread.join();

    // set the contactor to a safe state
    this->contactor_controller.set_target_state(ContactorState::CONTACTOR_OPEN);

    // If thread is active wait until it is terminated
    if (this->contactor_handling_thread.joinable())
        this->contactor_handling_thread.join();
}

void evse_board_supportImpl::handle_enable(bool& value) {
    // generate state A or state F
    double new_duty_cycle = value ? 100.0 : 0.0;
    // pause CP observation to avoid race condition between this thread and the CP observation thread
    this->disable_cp_observation();
    EVLOG_info << "handle_enable: Setting new duty cycle of " << std::fixed << std::setprecision(2) << new_duty_cycle
               << "%";
    this->pwm_controller.set_duty_cycle(new_duty_cycle);
    this->enable_cp_observation();
}

void evse_board_supportImpl::handle_pwm_on(double& value) {
    // pause CP observation to avoid race condition between this thread and the CP observation thread
    this->disable_cp_observation();
    EVLOG_info << "handle_pwm_on: Setting new duty cycle of " << std::fixed << std::setprecision(2) << value << "%";
    this->pwm_controller.set_duty_cycle(value);
    this->enable_cp_observation();
}

void evse_board_supportImpl::handle_pwm_off() {
    // generate state A
    double new_duty_cycle = 100.0;
    // pause CP observation to avoid race condition between this thread and the CP observation thread
    this->disable_cp_observation();
    EVLOG_info << "handle_pwm_off: Setting new duty cycle of " << std::fixed << std::setprecision(2) << new_duty_cycle
               << "%";
    this->pwm_controller.set_duty_cycle(100.0);
    this->enable_cp_observation();
}

void evse_board_supportImpl::handle_pwm_F() {
    // pause CP observation to avoid race condition between this thread and the CP observation thread
    this->disable_cp_observation();
    EVLOG_info << "Generating CP state F";
    this->pwm_controller.set_duty_cycle(0.0);
    this->enable_cp_observation();
}

void evse_board_supportImpl::handle_allow_power_on(types::evse_board_support::PowerOnOff& value) {

    if (value.allow_power_on && this->cp_current_state == types::cb_board_support::CPState::PilotFault) {
        EVLOG_warning << "Power on rejected due to detected pilot fault.";
        return;
    }

    if (evse_board_supportImpl::is_emergency) {
        EVLOG_warning << "Power on rejected due to detected emergency state.";
        return;
    }

    this->allow_power_on = value.allow_power_on;
}

void evse_board_supportImpl::handle_ac_switch_three_phases_while_charging(bool& value) {
    // your code for cmd ac_switch_three_phases_while_charging goes here
    (void)value;
}

void evse_board_supportImpl::handle_evse_replug(int& value) {
    // your code for cmd evse_replug goes here
    (void)value;
}

types::board_support_common::ProximityPilot evse_board_supportImpl::handle_ac_read_pp_ampacity() {
    // acquire lock to guard against possible background changes done by the observation thread
    std::lock_guard<std::mutex> lock(this->pp_observation_lock);

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

void evse_board_supportImpl::disable_cp_observation(void) {
    if (this->cp_observation_enabled) {
        EVLOG_debug << "Disabling CP observation";
        this->cp_observation_lock.lock();
        this->cp_observation_enabled = false;
    } else {
        EVLOG_debug << "Disabling CP observation (suppressed)";
    }
}

void evse_board_supportImpl::enable_cp_observation(void) {
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

void evse_board_supportImpl::cp_observation_worker(void) {
    double previous_duty_cycle {100.0};
    // both sides of the CP level
    struct cp_state_signal_side positive_side {
        types::cb_board_support::CPState::PilotFault, types::cb_board_support::CPState::PilotFault,
            types::cb_board_support::CPState::PilotFault, 0
    };
    struct cp_state_signal_side negative_side {
        types::cb_board_support::CPState::PilotFault, types::cb_board_support::CPState::PilotFault,
            types::cb_board_support::CPState::PilotFault, 0
    };

    EVLOG_info << "Control Pilot Observation Thread started";

    while (!this->termination_requested) {
        bool duty_cycle_changed {false};
        bool cp_state_changed {false};

        // acquire measurement lock for this loop round, wait for it eventually
        std::lock_guard<std::mutex> lock(this->cp_observation_lock);

        // do the actual measurement
        this->cp_controller.get_values(positive_side.voltage, negative_side.voltage);

        // positive signal side: map to CP state and check for changes
        positive_side.measured_state =
            this->cp_controller.voltage_to_state(positive_side.voltage, positive_side.current_state);
        cp_state_changed |= this->cp_state_changed(positive_side);

        // negative signal side: map to CP state and check for changes
        negative_side.measured_state =
            this->cp_controller.voltage_to_state(negative_side.voltage, negative_side.current_state);
        cp_state_changed |= this->cp_state_changed(negative_side);

        // at this point, the current_state member was already updated by the cp_state_changed methods

        // check whether we see a change of the duty cycle
        duty_cycle_changed = previous_duty_cycle != this->pwm_controller.get_duty_cycle();
        previous_duty_cycle = this->pwm_controller.get_duty_cycle();

        // if there was actually a change -> tell it to upper layers
        if (cp_state_changed || duty_cycle_changed) {
            // A diode fault is detected if the nominal duty cycle is set and the negative side of
            // the CP is not in state F (-12V)
            if (this->pwm_controller.is_nominal_duty_cycle() &&
                negative_side.current_state != types::cb_board_support::CPState::F) {
                if (!this->diode_fault_reported) {
                    this->diode_fault_reported = true;
                    this->update_cp_state_internally(types::cb_board_support::CPState::PilotFault, negative_side,
                                                     positive_side);
                    Everest::error::Error error_object = this->error_factory->create_error(
                        "evse_board_support/DiodeFault", "", "Diode fault detected.", Everest::error::Severity::High);
                    this->raise_error(error_object);
                }
                continue;
            }
        }

        if (cp_state_changed) {
            // in case we drive state F, then we cannot trust the peak detectors
            if (this->pwm_controller.get_duty_cycle() == 0.0) {
                types::board_support_common::BspEvent tmp {types::board_support_common::Event::F};
                this->publish_event(tmp);
                this->update_cp_state_internally(types::cb_board_support::CPState::F, negative_side, positive_side);
                continue;
            }
            // normal CP state change
            if (positive_side.current_state != this->cp_current_state) {
                // in case we see a pilot fault, we need to inform the upper layer
                if (positive_side.current_state == types::cb_board_support::CPState::PilotFault ||
                    negative_side.current_state == types::cb_board_support::CPState::PilotFault) {
                    if (this->pilot_fault_reported == false) {
                        Everest::error::Error error_object =
                            this->error_factory->create_error("evse_board_support/MREC14PilotFault", "",
                                                              "Pilot fault detected.", Everest::error::Severity::High);
                        this->raise_error(error_object);
                        this->pilot_fault_reported = true;
                        this->update_cp_state_internally(types::cb_board_support::CPState::PilotFault, negative_side,
                                                         positive_side);
                    }
                    continue;
                }
                // Before publishing CP state, we need to clear other errors if they were reported before
                if (this->diode_fault_reported) {
                    this->clear_error("evse_board_support/DiodeFault");
                    this->diode_fault_reported = false;
                }
                if (this->pilot_fault_reported) {
                    this->clear_error("evse_board_support/MREC14PilotFault");
                    this->pilot_fault_reported = false;
                }
                if (this->ventilation_fault_reported &&
                    positive_side.current_state != types::cb_board_support::CPState::D) {
                    this->clear_error("evse_board_support/VentilationNotAvailable");
                    this->ventilation_fault_reported = false;
                }

                // In case CP state changes from C to E (shorted CP to PE) it is possible that the positive side is
                // shortly in state D. The state D transition should be ignored in this case. This check must be done
                // before checking for the ventilation error to avoid false positives.
                if (positive_side.current_state == types::cb_board_support::CPState::D &&
                    negative_side.current_state == types::cb_board_support::CPState::E) {
                    this->update_cp_state_internally(types::cb_board_support::CPState::E, negative_side,
                                                     positive_side);
                    this->publish_event({types::board_support_common::Event::E});
                    continue;
                }
                // Check if a ventilation error has occurred
                if (positive_side.current_state == types::cb_board_support::CPState::D) {
                    // in case we see a ventilation request, although we do not support it,
                    // we need to inform the upper layer
                    Everest::error::Error error_object = this->error_factory->create_error(
                        "evse_board_support/VentilationNotAvailable", "", "Ventilation fault detected.",
                        Everest::error::Severity::High);
                    this->raise_error(error_object);
                    this->ventilation_fault_reported = true;
                    this->publish_event({types::board_support_common::Event::D});
                    this->update_cp_state_internally(positive_side.current_state, negative_side, positive_side);
                    continue;
                }

                try {
                    types::board_support_common::BspEvent tmp = cpstate_to_bspevent(positive_side.current_state);
                    this->publish_event(tmp);
                    this->update_cp_state_internally(positive_side.current_state, negative_side, positive_side);
                } catch (std::runtime_error& e) {
                    // Should never happen, when all invalid states are handled correctly
                    EVLOG_warning << e.what();
                }
            }
        }
    }

    EVLOG_info << "Control Pilot Observation Thread terminated";
}

void evse_board_supportImpl::contactor_handling_worker(void) {
    EVLOG_info << "Contactor Handling Thread started";

    while (!this->termination_requested) {
        ContactorState contactor_state {ContactorState::CONTACTOR_OPEN};

        if (evse_board_supportImpl::is_emergency)
            this->allow_power_on = false;

        // set the new contactor target state
        if (this->last_allow_power_on != this->allow_power_on) {
            this->last_allow_power_on = this->allow_power_on;

            EVLOG_info << (this->allow_power_on ? "Closing" : "Opening") << " contactor...";

            this->contactor_controller.set_target_state(this->allow_power_on ? ContactorState::CONTACTOR_CLOSED
                                                                             : ContactorState::CONTACTOR_OPEN);

        } else if ((this->contactor_controller.get_delay_contactor_close() == false) &&
                   (((this->mod->config.contactor_1_feedback_type == "none") &&
                     (this->mod->config.contactor_2_feedback_type == "none")) ||
                    ((this->mod->config.contactor_1_feedback_type == "none") &&
                     (this->contactor_controller.get_target_phase_count() == 1)))) {
            // if there is no new target states to set, we should monitor the contactor feedback through edge events.
            // If no new edge events for contactor feedback are expected because the relays have no feedback
            // connected to them, and we are not intentionally delaying switching on, we should simulate the
            // actual state and send the corresponding events to the higher layer. Note that his should only happen if:
            // - Either both contactors have no feedback, because if at least one of them has feedback, the waiting
            //   mechanism for edge events should take place
            // - OR if we are in single phase operation and the first contactor has no feedback

            // in case a new target state is set, we need to update the actual state by simply setting
            // it to be equal to the required target state
            if (this->contactor_controller.is_error_state()) {
                this->contactor_controller.set_actual_state(
                    this->contactor_controller.get_state(StateType::TARGET_STATE));
                EVLOG_info << "Contactor state: " << this->contactor_controller.get_state(StateType::TARGET_STATE);

                // publish PowerOn or PowerOff event
                types::board_support_common::Event tmp_event =
                    (this->contactor_controller.get_state(StateType::TARGET_STATE) == ContactorState::CONTACTOR_CLOSED)
                        ? types::board_support_common::Event::PowerOn
                        : types::board_support_common::Event::PowerOff;
                types::board_support_common::BspEvent tmp {tmp_event};
                this->publish_event(tmp);
            }

            // intentional sleep because no feedback monitoring is needed
            std::this_thread::sleep_for(200ms);

        } else if (this->contactor_controller.wait_for_events(20ms) == true) {
            // otherwise wait for edge events to happen on one or both contactors

            // Read if the new event is a contactor closed or opened event
            contactor_state = this->contactor_controller.read_events() ? ContactorState::CONTACTOR_CLOSED
                                                                       : ContactorState::CONTACTOR_OPEN;

            // if it is a new event, set the actual state
            if (contactor_state == this->contactor_controller.get_state(StateType::TARGET_STATE) &&
                contactor_state != this->contactor_controller.get_state(StateType::ACTUAL_STATE)) {
                // if the received state is equal to the expected state, set the actual state
                this->contactor_controller.set_actual_state(contactor_state);
                EVLOG_info << "Contactor state: " << this->contactor_controller.get_state(StateType::TARGET_STATE);

                // publish PowerOn or PowerOff event
                types::board_support_common::Event tmp_event = (contactor_state == ContactorState::CONTACTOR_CLOSED)
                                                                   ? types::board_support_common::Event::PowerOn
                                                                   : types::board_support_common::Event::PowerOff;
                types::board_support_common::BspEvent tmp {tmp_event};
                this->publish_event(tmp);
            }

        } else {
            // if we are actually waiting for a new state but it has not been detected yet
            if (this->contactor_controller.is_error_state()) {

                // if no contactor feedback has been detected but we are intentionally delaying switching on
                // the contactor to prevent wearout. Contactor is only allowed to be switched on once every 10 seconds
                if ((this->contactor_controller.is_switch_on_allowed() == true) &&
                    (this->contactor_controller.get_delay_contactor_close() == true) &&
                    (this->contactor_controller.get_state(StateType::TARGET_STATE) ==
                     ContactorState::CONTACTOR_CLOSED)) {
                    // set the target state again to switch on the contactor
                    this->contactor_controller.set_target_state(ContactorState::CONTACTOR_CLOSED);

                    this->contactor_controller.reset_delay_contactor_close();
                }

                else if ((this->contactor_controller.get_delay_contactor_close() == true) &&
                         (((this->mod->config.contactor_1_feedback_type == "none") &&
                           (this->mod->config.contactor_2_feedback_type == "none")) ||
                          ((this->mod->config.contactor_1_feedback_type == "none") &&
                           (this->contactor_controller.get_target_phase_count() == 1)))) {
                    // intentional sleep because no feedback monitoring is needed and we know that we are waiting
                    // for the 10 seconds timeout regarding relay wear prevention to pass
                    std::this_thread::sleep_for(200ms);
                }

                else if ((this->contactor_controller.get_delay_contactor_close() == false) &&
                         (this->contactor_controller.get_is_new_target_state_set() == true) &&
                         (std::chrono::steady_clock::now() - this->contactor_controller.get_new_target_state_ts() >
                          this->contactor_feedback_timeout)) {
                    // if no new events for contactor feedback have been detected for 200ms after a new
                    // target state was set
                    EVLOG_error << "Failed to detect feedback. Contactor should be: "
                                << this->contactor_controller.get_state(StateType::TARGET_STATE);

                    // publish a 'contactor fault' event to the upper layer. This fault is critical as it might be a
                    // sign of a hardware issue, therefore the fault will not be cleared to prevent further damage.
                    Everest::error::Error error_object = this->error_factory->create_error(
                        "evse_board_support/MREC17EVSEContactorFault", "", "Unexpected contactor feedback",
                        Everest::error::Severity::High);
                    this->raise_error(error_object);

                    // reset the flag that marked the start of counting the contactor_feedback_timeout
                    this->contactor_controller.reset_is_new_target_state_set();
                }
            }
        }
    }

    EVLOG_info << "Contactor Handling Thread terminated";
}

} // namespace evse_board_support
} // namespace module
