// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "connector_lockImpl.hpp"

using namespace std::chrono_literals;

namespace module {
namespace connector_lock {

void connector_lockImpl::init() {

    this->lock_actuator = CbLockActuator(this->mod->config.drv8872_in1_gpio_line_name,
                                         this->mod->config.drv8872_in2_gpio_line_name,
                                         this->mod->config.drv8872_in1_active_low,
                                         this->mod->config.drv8872_in2_active_low);
    // CbLockSense object for motors
    this->lock_sense = CbLockSense(this->mod->config.sense_adc_device,
                                   this->mod->config.sense_adc_channel,
                                   this->mod->config.unlocked_threshold_voltage_min,
                                   this->mod->config.unlocked_threshold_voltage_max,
                                   this->mod->config.locked_threshold_voltage_min,
                                   this->mod->config.locked_threshold_voltage_max);
    // CbCapSense object
    this->cap_sense = CbCapSense(this->mod->config.capcharge_adc_device,
                                this->mod->config.capcharge_adc_channel,
                                this->mod->config.charged_threshold_voltage);
}

void connector_lockImpl::ready() {

    // unlock plug lock at start time
    // wait for caps are loaded. Simulate voltage value for full charge > 10V
    // TODO add timeout
    while(!this->cap_sense.is_charged()) {
        EVLOG_info << "Current capacitor voltage: " << this->cap_sense.get_voltage() << "mV";
        std::this_thread::sleep_for(100ms);
    }
    EVLOG_info << "Current capacitor voltage: " << this->cap_sense.get_voltage() << "mV";
    this->handle_unlock();
}

void connector_lockImpl::handle_lock() {
    this->lock_actuator.backward();
    std::this_thread::sleep_for(std::chrono::milliseconds(this->mod->config.actuator_duration));
    this->lock_actuator.brake();

    int feedback_voltage = lock_sense.get_voltage();

    if (this->lock_sense.is_locked()) {
        EVLOG_info << "plug is locked. Feedback voltage: " << feedback_voltage << " mV";
    } else {
        // TODO error handling
        EVLOG_warning << "plug is not locked. Feedback voltage: " << feedback_voltage << " mV";
    }
}

void connector_lockImpl::handle_unlock() {
    this->lock_actuator.forward();
    std::this_thread::sleep_for(std::chrono::milliseconds(this->mod->config.actuator_duration));
    this->lock_actuator.brake();

    int feedback_voltage = this->lock_sense.get_voltage();

    if (this->lock_sense.is_unlocked()) {
        EVLOG_info << "plug is unlocked. Feedback voltage: " << feedback_voltage << " mV";
    } else {
        // TODO error handling
        EVLOG_warning << "plug is not unlocked. Feedback voltage: " << feedback_voltage << " mV";
    }
}

} // namespace connector_lock
} // namespace module
