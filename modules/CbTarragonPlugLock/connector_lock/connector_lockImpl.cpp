// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "connector_lockImpl.hpp"

namespace module {
namespace connector_lock {

void connector_lockImpl::init() {
    
    this->duration = this->mod->config.actuator_duration;
    this->lock_actuator = CbLockActuator(this->mod->config.drv8872_in1_gpio_line_name,
                                         this->mod->config.drv8872_in2_gpio_line_name,
                                         this->mod->config.drv8872_in1_active_low,
                                         this->mod->config.drv8872_in2_active_low);

    this->lock_sense = CbLockSense(this->mod->config.sense_adc_device,
                                   this->mod->config.sense_adc_channel,
                                   700,
                                   2900);
}

void connector_lockImpl::ready() {
}

void connector_lockImpl::handle_lock() {
    // your code for cmd lock goes here
}

void connector_lockImpl::handle_unlock() {
    // your code for cmd unlock goes here
}

} // namespace connector_lock
} // namespace module
