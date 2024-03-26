// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include <PWM.hpp>
#include <PWMChip.hpp>

#include "CbTarragonDIPWM.hpp"

namespace module {

void CbTarragonDIPWM::configure_digital_input_pwm() {
    // note: this will throw a std::runtime_error if it fails to find the PWM,
    // so no need for logging
    PWM pwm = PWMChip::find_pwm(this->device, this->channel);

    pwm.set_period(this->PWM_PERIOD);
    pwm.set_duty_cycle(this->threshold_voltage * this->PWM_PERIOD / this->MAX_THRESHOLD_VOLTAGE);
    if (not pwm.is_enabled())
        pwm.set_enabled(true);
}

} // namespace module
