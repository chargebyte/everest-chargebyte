#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <gpiod.hpp>
#include <PWM.hpp>
#include <PWMChip.hpp>
#include <gpiodUtils.hpp>
#include "CbTarragonPWM.hpp"

CbTarragonPWM::CbTarragonPWM(void) {
}

CbTarragonPWM::CbTarragonPWM(const std::string& pwm_device, unsigned int pwm_device_channel,
                             const std::string& invert_gpioline_name) :
    pwm(PWMChip::find_pwm(pwm_device, pwm_device_channel)) {

    this->cp_invert = std::make_unique<gpiod::line_request> (get_gpioline_by_name(invert_gpioline_name,
                                                                                  "CbTarragonPWM",
                                                                                  gpiod::line_settings()
                                                                                      .set_direction(gpiod::line::direction::OUTPUT)
                                                                                      .set_output_value(gpiod::line::value::INACTIVE)));

    // in case we find it already enabled -> disable to reconfigure
    if (this->pwm.is_enabled())
        this->pwm.set_enabled(false);

    // set frequency to 1 kHz and preset duty cycle to 100%
    this->pwm.set_frequency_and_ratio(1000, 1);
}

double CbTarragonPWM::get_duty_cycle(void) {
    double rv = 0.0;

    if (this->get_cp_invert() && this->pwm.is_enabled())
        rv = this->pwm.get_ratio() * 100.0;

    return rv;
}

void CbTarragonPWM::set_duty_cycle(double duty_cycle) {
    this->pwm.set_ratio(duty_cycle / 100.0);

    // if not yet enable, enable the PWM output
    if (!this->pwm.is_enabled())
        this->pwm.set_enabled(true);

    // ensure that CP_INVERT is driven
    if (!this->get_cp_invert())
        this->set_cp_invert(true);
}

bool CbTarragonPWM::is_enabled(void) {
    return this->get_cp_invert();
}

void CbTarragonPWM::disable(void) {
    // generate 100% for at least 1 cycle for a deterministic falling edge
    this->set_duty_cycle(100.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    this->set_cp_invert(false);
    this->pwm.set_enabled(false);
}

bool CbTarragonPWM::get_cp_invert(void) {
    return this->cp_invert->get_value(this->cp_invert->offsets()[0]) == gpiod::line::value::ACTIVE;
}

void CbTarragonPWM::set_cp_invert(bool active) {
    this->cp_invert->set_value(this->cp_invert->offsets()[0],
                               active
                                   ? gpiod::line::value::ACTIVE
                                   : gpiod::line::value::INACTIVE);
}
