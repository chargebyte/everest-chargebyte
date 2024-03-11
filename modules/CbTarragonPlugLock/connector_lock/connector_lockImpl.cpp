// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "connector_lockImpl.hpp"
#include <chrono>

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

bool connector_lockImpl::wait_for_charged(std::chrono::seconds timeout) {

    auto now = std::chrono::steady_clock::now();

    while (!this->cap_sense.is_charged()) {
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - now) > timeout) {
            EVLOG_warning << "Timeout capacitor voltage: " << this->cap_sense.get_voltage() << " mV";
            return false;
        }
        std::this_thread::sleep_for(100ms);
    };

    return true;
}

void connector_lockImpl::ready() {

    // unlock plug lock at start time
    // wait for caps are loaded. Simulate voltage value for full charge > 10V
    if (this->wait_for_charged(CHARGED_TIMEOUT_INITIAL) == false)
        throw std::runtime_error("Initial capacitor voltage not reached");
    this->handle_unlock();
}

void connector_lockImpl::handle_lock() {
    // wait for caps are loaded before locking pluglock
    if (this->wait_for_charged(CHARGED_TIMEOUT_WORK) == false)
        EVLOG_error << "Capacitor voltage not reached before lock";

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
    // wait for caps are loaded before unlocking pluglock
    if (this->wait_for_charged(CHARGED_TIMEOUT_WORK) == false)
        EVLOG_error << "Capacitor voltage not reached before unlock";

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
