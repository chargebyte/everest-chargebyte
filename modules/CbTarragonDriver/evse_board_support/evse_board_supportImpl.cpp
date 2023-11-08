// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "evse_board_supportImpl.hpp"

namespace module {
namespace evse_board_support {

void evse_board_supportImpl::init() {
    // just for clarity
    this->termination_requested = false;

    // Configure hardware capabilities
    this->hw_capabilities.max_current_A_import = 16;
    this->hw_capabilities.min_current_A_import = 0;
    this->hw_capabilities.max_phase_count_import = 3;
    this->hw_capabilities.min_phase_count_import = 1;
    this->hw_capabilities.max_current_A_export = 16;
    this->hw_capabilities.min_current_A_export = 0;
    this->hw_capabilities.max_phase_count_export = 3;
    this->hw_capabilities.min_phase_count_export = 1;

    // Control Pilot state observation
    this->cp_controller = CbTarragonCP(this->mod->config.cp_pos_peak_adc_device,
                                       this->mod->config.cp_pos_peak_adc_channel,
                                       this->mod->config.cp_rst_pos_peak_gpio_line_name,
                                       this->mod->config.cp_neg_peak_adc_device,
                                       this->mod->config.cp_neg_peak_adc_channel,
                                       this->mod->config.cp_rst_neg_peak_gpio_line_name);

    // Control Pilot PWM generatation
    this->pwm_controller = CbTarragonPWM(this->mod->config.cp_pwm_device,
                                         this->mod->config.cp_pwmchannel,
                                         this->mod->config.cp_invert_gpio_line_name);

    // acquire the lock so that the CP observation does not start immediately
    this->cp_observation_lock.lock();
    this->cp_observation_enabled = false;

    // preset with PowerOn for nice logging
    this->cp_current_state = types::board_support_common::BspEvent{types::board_support_common::Event::PowerOn};

    // start cp-observation-worker thread
    this->cp_observation_thread = std::thread(&evse_board_supportImpl::cp_observation_worker, this);

    // Proximity Pilot state observation
    this->pp_controller = CbTarragonPP(this->mod->config.pp_adc_device,
                                       this->mod->config.pp_adc_channel);
}

void evse_board_supportImpl::ready() {
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
}

void evse_board_supportImpl::handle_setup(bool& three_phases, bool& has_ventilation, std::string& country_code) {
    // your code for cmd setup goes here
}

types::evse_board_support::HardwareCapabilities evse_board_supportImpl::handle_get_hw_capabilities() {
    return this->hw_capabilities;
}

void evse_board_supportImpl::handle_enable(bool& value) {
    // generate state A or state F
    double new_duty_cycle = value ? 100.0 : 0.0;

    EVLOG_info << "handle_enable: Setting new duty cycle of " << std::fixed << std::setprecision(2) << new_duty_cycle << "%";
    this->pwm_controller.set_duty_cycle(new_duty_cycle);

    this->enable_cp_observation();
}

void evse_board_supportImpl::handle_pwm_on(double& value) {
    EVLOG_info << "handle_pwm_on: Setting new duty cycle of " << std::fixed << std::setprecision(2) << value << "%";
    this->pwm_controller.set_duty_cycle(value);

    this->enable_cp_observation();
}

void evse_board_supportImpl::handle_pwm_off() {
    // generate state A
    double new_duty_cycle = 100.0;

    EVLOG_info << "handle_pwm_off: Setting new duty cycle of " << std::fixed << std::setprecision(2) << new_duty_cycle << "%";
    this->pwm_controller.set_duty_cycle(100.0);

    this->enable_cp_observation();
}

void evse_board_supportImpl::handle_pwm_F() {
    EVLOG_info << "Generating CP state F";
    this->pwm_controller.set_duty_cycle(0.0);

    this->enable_cp_observation();
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

        EVLOG_info << "Read PP ampacity value: " << this->pp_ampacity.ampacity <<
                  " (U_PP: " << voltage << " mV)";

        // reset possible set flag since we successfully read a valid value
        this->pp_fault_reported = false;
    }
    catch (std::underflow_error& e) {
        EVLOG_error << e.what();

        // publish a ProximityFault
        types::board_support_common::BspEvent tmp{types::board_support_common::Event::MREC_23_ProximityFault};
        this->publish_event(tmp);

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

            // reset possible set flag since we successfully read a valid value
            this->pp_fault_reported = false;

            if (this->pp_ampacity.ampacity != prev_value) {

                if (this->pp_ampacity.ampacity == types::board_support_common::Ampacity::None) {
                    EVLOG_info << "PP noticed plug removal from socket (U_PP: " << voltage << " mV)";
                } else {
                    EVLOG_info << "PP ampacity change from " << prev_value << " to " << this->pp_ampacity.ampacity <<
                                  " (U_PP: " << voltage << " mV)";
                }

                // publish new value, upper layer should decide how to handle the change
                this->publish_pp_ampacity(this->pp_ampacity);
            }
        }
        catch (std::underflow_error& e) {
            if (!this->pp_fault_reported) {
                EVLOG_error << e.what();

                // publish a ProximityFault
                types::board_support_common::BspEvent tmp{types::board_support_common::Event::MREC_23_ProximityFault};
                this->publish_event(tmp);

                this->pp_fault_reported = true;
            }
        }

        this->pp_observation_lock.unlock();

        // break before sleeping in case of already requested termination
        if (this->termination_requested)
            break;

        // let's sleep for a "randomly" selected time: 500ms should be a good trade-off between
        // CPU usage and fast recognition of any kind of trouble with the PP line
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    EVLOG_info << "Proximity Pilot Observation Thread terminated";
}

void evse_board_supportImpl::handle_ac_set_overcurrent_limit_A(double& value) {
    // your code for cmd ac_set_overcurrent_limit_A goes here
}

void evse_board_supportImpl::disable_cp_observation(void) {
    if (this->cp_observation_enabled) {
        EVLOG_info << "Disabling CP observation";
        this->cp_observation_lock.lock();
        this->cp_observation_enabled = false;
    } else {
        EVLOG_debug << "Disabling CP observation (suppressed)";
    }
}

void evse_board_supportImpl::enable_cp_observation(void) {
    if (!this->cp_observation_enabled) {
        EVLOG_info << "Enabling CP observation";
        this->cp_observation_enabled = true;
        this->cp_observation_lock.unlock();
    } else {
        EVLOG_debug << "Enabling CP observation (suppressed)";
    }
}

struct cp_state_signal_side {
    /// @brief previous state is what we measured before the last round
    types::board_support_common::Event previous_state;

    /// @brief current state is what we measured in the last round
    types::board_support_common::Event current_state;

    /// @brief measured state is what we just measured in this round
    types::board_support_common::Event measured_state;

    /// @brief the voltage of the just completed measurement
    int voltage;
};

bool evse_board_supportImpl::cp_state_changed(struct cp_state_signal_side& signal_side) {
    bool rv{false};

    // CP state is only detected if the new state is different from the previous one (first condition).
    // Additionaly, to filter simple disturbances, a new state must be detected twice before notifing it (second condition).
    // For that, we need at least two CP state measurements (third condition)

    if (signal_side.previous_state != signal_side.measured_state &&
        signal_side.current_state == signal_side.measured_state &&
        this->cp_controller.is_valid_cp_state(signal_side.measured_state)) {

        // update the previous state
        signal_side.previous_state = signal_side.current_state;
        rv = true;

    } else if (signal_side.measured_state == signal_side.previous_state &&
               signal_side.measured_state != signal_side.current_state) {
        EVLOG_warning << "CP state change from " << signal_side.previous_state << " to "
                      << signal_side.current_state << " suppressed";
    }

    signal_side.current_state = signal_side.measured_state;

    return rv;
}

void evse_board_supportImpl::cp_observation_worker(void) {
    // both sides of the CP level
    struct cp_state_signal_side positive_side{types::board_support_common::Event::MREC_14_PilotFault,
                                              types::board_support_common::Event::MREC_14_PilotFault,
                                              types::board_support_common::Event::MREC_14_PilotFault,
                                              0};
    struct cp_state_signal_side negative_side{types::board_support_common::Event::MREC_14_PilotFault,
                                              types::board_support_common::Event::MREC_14_PilotFault,
                                              types::board_support_common::Event::MREC_14_PilotFault,
                                              0};

    EVLOG_info << "Control Pilot Observation Thread started";

    while (!this->termination_requested) {
        bool cp_state_changed{false};

        // acquire measurement lock for this loop round, wait for it eventually
        std::lock_guard<std::mutex> lock(this->cp_observation_lock);

        // do the actual measurement
        this->cp_controller.get_values(positive_side.voltage, negative_side.voltage);

        // positive signal side: map to CP state and check for changes
        positive_side.measured_state = this->cp_controller.voltage_to_state(positive_side.voltage, positive_side.current_state);
        cp_state_changed |= this->cp_state_changed(positive_side);

        // negative signal side: map to CP state and check for changes
        negative_side.measured_state = this->cp_controller.voltage_to_state(negative_side.voltage, negative_side.current_state);
        cp_state_changed |= this->cp_state_changed(negative_side);

        // at this point, the current_state member was already updated by the cp_state_changed methods

        // if there was actually a change -> tell it to upper layers
        if (cp_state_changed) {
            // check whether the PWM is actively driven by us
            if (this->pwm_controller.get_duty_cycle() > 0.0 &&
                this->pwm_controller.get_duty_cycle() < 100.0 &&
                // check wether -12 V was seen on the negative side
                negative_side.current_state != types::board_support_common::Event::F) {
                types::board_support_common::BspEvent tmp {types::board_support_common::Event::ErrorDF};
                this->publish_event(tmp);
                EVLOG_warning << "Diode fault detected. Previous CP state was: " << this->cp_current_state.load().event << ", "
                              << "U_CP+: " << positive_side.voltage << " mV, "
                              << "U_CP-: " << negative_side.voltage << " mV";
                this->cp_current_state = tmp;
                continue;
            }

            // in case we drive state F, then we cannot trust the peak detectors
            if (this->pwm_controller.get_duty_cycle() == 0.0) {
                types::board_support_common::BspEvent tmp {types::board_support_common::Event::F};
                this->publish_event(tmp);
                EVLOG_info << "CP state change from " << this->cp_current_state.load().event << " to " << types::board_support_common::Event::F << ", "
                           << "U_CP+: " << positive_side.voltage << " mV, "
                           << "U_CP-: " << negative_side.voltage << " mV";
                this->cp_current_state = tmp;
                continue;
            }

            // normal CP state change
            if (positive_side.current_state != this->cp_current_state.load().event) {
                types::board_support_common::BspEvent tmp {positive_side.current_state};
                this->publish_event(tmp);
                EVLOG_info << "CP state change from " << this->cp_current_state.load().event << " to " << positive_side.current_state << ", "
                           << "U_CP+: " << positive_side.voltage << " mV, "
                           << "U_CP-: " << negative_side.voltage << " mV";
                this->cp_current_state = tmp;
            }
        }
    }

    EVLOG_info << "Control Pilot Observation Thread terminated";
}

} // namespace evse_board_support
} // namespace module
