#include "CbLockActuator.hpp"
#include "gpiodUtils.hpp"

CbLockActuator::CbLockActuator(void) {
}

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
