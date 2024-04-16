// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "connector_lockImpl.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace module {
namespace connector_lock {

void connector_lockImpl::init() {

    this->assumed_is_locked = false;

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

    if (this->mod->config.enable_monitoring) {
        this->lock_observation_thread = std::thread(&connector_lockImpl::lock_observation_worker, this);
    }
}

void connector_lockImpl::lock_observation_worker(void) {
    EVLOG_debug << "Running observation worker";

    bool error_closed_raised = false;
    bool error_opened_raised = false;

    while (true) {
        if (this->lock_observation_thread.shouldExit()) {
            break;
        }

        std::this_thread::sleep_for(FEEDBACK_CHECK_INTERVAL);

        // lock against driving while observing
        std::scoped_lock lock(this->observation_mtx);

        EVLOG_verbose << "Plug lock voltage is " << this->lock_sense.get_voltage() << " mV";

        // if mismatch on is_locked, and not already raised
        if (this->assumed_is_locked and not this->lock_sense.is_locked() and not error_opened_raised) {
            this->raise_connector_lock_ConnectorLockUnexpectedOpen("Plug lock unexpectedly opened", Everest::error::Severity::High);
            error_opened_raised = true;
        } else if (this->assumed_is_locked and this->lock_sense.is_locked() and error_opened_raised) {
            this->request_clear_all_connector_lock_ConnectorLockUnexpectedOpen();
            error_opened_raised = false;
        }

        // if mismatch on is_unlocked, and not already raised
        if (not this->assumed_is_locked and not this->lock_sense.is_unlocked() and not error_closed_raised) {
            this->raise_connector_lock_ConnectorLockUnexpectedClose("Plug lock unexpectedly closed", Everest::error::Severity::High);
            error_closed_raised = true;
        } else if (not this->assumed_is_locked and this->lock_sense.is_unlocked() and error_closed_raised) {
            this->request_clear_all_connector_lock_ConnectorLockUnexpectedClose();
            error_closed_raised = false;
        }
    }
}

bool connector_lockImpl::wait_for_charged(std::chrono::seconds timeout) {

    auto start_time = std::chrono::steady_clock::now();

    while (!this->cap_sense.is_charged()) {
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time) > timeout) {
            EVLOG_warning << "Timeout! Measured capacitor voltage: " << this->cap_sense.get_voltage() << " mV," <<
            		         "expected capacitor voltage: " << this->cap_sense.get_threshold_voltage() << " mV";
            return false;
        }
        std::this_thread::sleep_for(100ms);
    };

    return true;
}

void connector_lockImpl::ready() {
    // unlock plug lock at start time
    // wait for caps are loaded.
    if (this->wait_for_charged(CHARGED_TIMEOUT_INITIAL) == false) {
        this->raise_connector_lock_ConnectorLockCapNotCharged("Initial capacitor voltage not reached", Everest::error::Severity::High);
    }

    this->handle_unlock();
}

void connector_lockImpl::handle_lock() {
    // wait for caps are loaded before locking pluglock
    if (this->wait_for_charged(CHARGED_TIMEOUT_WORK) == false) {
        this->raise_connector_lock_ConnectorLockCapNotCharged("Capacitor voltage not reached before lock", Everest::error::Severity::Medium);
    } else {
        this->request_clear_all_connector_lock_ConnectorLockCapNotCharged();
    }

    // lock against observation while trying to drive
    std::scoped_lock lock(this->observation_mtx);

    this->lock_actuator.backward();
    std::this_thread::sleep_for(std::chrono::milliseconds(this->mod->config.actuator_duration));
    this->lock_actuator.brake();

    int feedback_voltage = this->lock_sense.get_voltage();

    if (this->lock_sense.is_locked()) {
        EVLOG_info << "Plug is locked. Feedback voltage: " << feedback_voltage << " mV";
        this->request_clear_all_connector_lock_ConnectorLockFailedLock();
        this->assumed_is_locked = true;
    } else {
        EVLOG_warning << "Plug is not locked. Feedback voltage: " << feedback_voltage << " mV";
        this->raise_connector_lock_ConnectorLockFailedLock("Plug is not locked", Everest::error::Severity::Medium);
        this->assumed_is_locked = false;
    }
}

void connector_lockImpl::handle_unlock() {
    // wait for caps are loaded before unlocking pluglock
    if (this->wait_for_charged(CHARGED_TIMEOUT_WORK) == false) {
        this->raise_connector_lock_ConnectorLockCapNotCharged("Capacitor voltage not reached before unlock", Everest::error::Severity::Medium);
    } else {
        this->request_clear_all_connector_lock_ConnectorLockCapNotCharged();
    }

    // lock against observation while trying to drive
    std::scoped_lock lock(this->observation_mtx);

    this->lock_actuator.forward();
    std::this_thread::sleep_for(std::chrono::milliseconds(this->mod->config.actuator_duration));
    this->lock_actuator.brake();

    int feedback_voltage = this->lock_sense.get_voltage();

    if (this->lock_sense.is_unlocked()) {
        EVLOG_info << "Plug is unlocked. Feedback voltage: " << feedback_voltage << " mV";
        this->request_clear_all_connector_lock_ConnectorLockFailedUnlock();
        this->assumed_is_locked = false;
    } else {
        EVLOG_warning << "Plug is not unlocked. Feedback voltage: " << feedback_voltage << " mV";
        this->raise_connector_lock_ConnectorLockFailedUnlock("Plug is not unlocked", Everest::error::Severity::Medium);
        this->assumed_is_locked = true;
    }
}

connector_lockImpl::~connector_lockImpl() {
    // if thread is active, stop it and wait until it is terminated
    this->lock_observation_thread.stop();
}

} // namespace connector_lock
} // namespace module
