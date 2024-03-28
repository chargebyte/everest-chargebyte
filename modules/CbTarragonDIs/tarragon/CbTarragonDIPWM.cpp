// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include <everest/logging.hpp>
#include <PWM.hpp>
#include <PWMChip.hpp>

#include "CbTarragonDIPWM.hpp"

namespace module {

void CbTarragonDIPWM::configure_digital_input_pwm() {
    pwm.set_period(this->PWM_PERIOD);
    auto duty_cycle = this->threshold_voltage * this->PWM_PERIOD / this->MAX_THRESHOLD_VOLTAGE;
    pwm.set_duty_cycle(duty_cycle);
    pwm.set_enabled(true);
    EVLOG_info << "Enabled digital input reference PWM with period " << this->PWM_PERIOD.count() << " and duty cycle "
               << duty_cycle.count();
}

} // namespace module
