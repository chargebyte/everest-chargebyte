// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "connector_lockImpl.hpp"

using namespace std::chrono_literals;

namespace module {
namespace connector_lock {

void connector_lockImpl::init() {

    this->actuator_duration = this->mod->config.actuator_duration;
    // range check for actuator duration
    if (this->actuator_duration <= 0 || this->actuator_duration > 4000)
        throw std::out_of_range("Configured actuator duration (" + std::to_string(this->actuator_duration) +
                                " ms) is out of allowed range (1-4000 ms)");

    this->lock_actuator = CbLockActuator(this->mod->config.drv8872_in1_gpio_line_name,
                                         this->mod->config.drv8872_in2_gpio_line_name,
                                         this->mod->config.drv8872_in1_active_low,
                                         this->mod->config.drv8872_in2_active_low);
    // CbLockSense object for KUESTER02S motors
    this->lock_sense = CbLockSense(this->mod->config.sense_adc_device,
                                   this->mod->config.sense_adc_channel,
                                   2900, // unlock threshold min in mV
                                   3300, // unlock threshold max in mV
                                   0, // lock threshold min in mV
                                   700); // lock threshold max in mV
}

void connector_lockImpl::ready() {

    EVLOG_info << "Motortype KUESTER02S";

    // unlock plug lock at start time
    // wait for caps are loaded. Simulate voltage value for full charge > 10V
    // TODO wait depending on cap voltage read dynamically
    std::this_thread::sleep_for(5000ms);
    this->handle_unlock();
}

void connector_lockImpl::handle_lock() {
    this->lock_actuator.backward();
    std::this_thread::sleep_for(std::chrono::milliseconds(this->actuator_duration));
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
    std::this_thread::sleep_for(std::chrono::milliseconds(this->actuator_duration));
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
