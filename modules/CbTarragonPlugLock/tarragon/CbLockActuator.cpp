// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include "CbLockActuator.hpp"
#include "gpiodUtils.hpp"

CbLockActuator::CbLockActuator(void) {
}

// Initial state of Motordriver is coast -> charge caps
CbLockActuator::CbLockActuator(const std::string& drv8872_in1_gpio_line_name,
                               const std::string& drv8872_in2_gpio_line_name, bool drv8872_in1_active_low,
                               bool drv8872_in2_active_low) {
    this->motor_in1 = std::make_unique<gpiod::line_request> (get_gpioline_by_name(drv8872_in1_gpio_line_name,
                                                                                  "CbLockActuator",
                                                                                  gpiod::line_settings()
                                                                                      .set_direction(gpiod::line::direction::OUTPUT)
                                                                                      .set_active_low(drv8872_in1_active_low)
                                                                                      .set_output_value(gpiod::line::value::INACTIVE)));

    this->motor_in2 = std::make_unique<gpiod::line_request> (get_gpioline_by_name(drv8872_in2_gpio_line_name,
                                                                                  "CbLockActuator",
                                                                                  gpiod::line_settings()
                                                                                      .set_direction(gpiod::line::direction::OUTPUT)
                                                                                      .set_active_low(drv8872_in2_active_low)
                                                                                      .set_output_value(gpiod::line::value::INACTIVE)));
}

void CbLockActuator::forward(void) {
    this->motor_in1->set_value(this->motor_in1->offsets()[0], gpiod::line::value::ACTIVE);
    this->motor_in2->set_value(this->motor_in2->offsets()[0], gpiod::line::value::INACTIVE);
}

void CbLockActuator::backward(void) {
    this->motor_in1->set_value(this->motor_in1->offsets()[0], gpiod::line::value::INACTIVE);
    this->motor_in2->set_value(this->motor_in2->offsets()[0], gpiod::line::value::ACTIVE);
}

void CbLockActuator::coast(void) {
    this->motor_in1->set_value(this->motor_in1->offsets()[0], gpiod::line::value::INACTIVE);
    this->motor_in2->set_value(this->motor_in2->offsets()[0], gpiod::line::value::INACTIVE);
}

void CbLockActuator::brake(void) {
    this->motor_in1->set_value(this->motor_in1->offsets()[0], gpiod::line::value::ACTIVE);
    this->motor_in2->set_value(this->motor_in2->offsets()[0], gpiod::line::value::ACTIVE);
}
