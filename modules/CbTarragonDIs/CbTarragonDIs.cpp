// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include <PWM.hpp>
#include <PWMChip.hpp>

#include "CbTarragonDIs.hpp"
#include "configuration.h"

namespace module {

void CbTarragonDIs::init() {
    invoke_init(*p_empty);

    EVLOG_info << PROJECT_DESCRIPTION << " (version: " << PROJECT_VERSION << ")";

    this->configure_digital_input_pwm();
}

void CbTarragonDIs::ready() {
    invoke_ready(*p_empty);
}

void CbTarragonDIs::configure_digital_input_pwm() {
    // note: this will throw a std::runtime_error if it fails to find the PWM,
    // so no need for logging
    PWM pwm = PWMChip::find_pwm(this->config.pwm_device, this->config.pwmchannel);

    pwm.set_period(this->PWM_PERIOD);
    pwm.set_duty_cycle(this->config.threshold_voltage * this->PWM_PERIOD / this->MAX_THRESHOLD_VOLTAGE);
    if (not pwm.is_enabled())
        pwm.set_enabled(true);
}

} // namespace module
