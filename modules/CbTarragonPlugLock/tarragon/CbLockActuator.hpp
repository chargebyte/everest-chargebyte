// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include <string>
#include <gpiod.hpp>

//
//  A class for abstracting the plug lock motor control
//
class CbLockActuator {

public:
    /// @brief Default constructor.
    CbLockActuator(void);

    /// @brief Constructor.
    /// @param drv8872_in1_gpio_line_name The GPIO line name which controls the motor driver pin 'in1'
    /// @param drv8872_in2_gpio_line_name The GPIO line name which controls the motor driver pin 'in2'
    /// @param drv8872_in1_active_low The GPIO polarity of motor driver pin 'in1' (active low = true, active high =
    /// false)
    /// @param drv8872_in2_active_low The GPIO polarity of motor driver pin 'in2' (active low = true, active high =
    /// false)
    CbLockActuator(const std::string& drv8872_in1_gpio_line_name, const std::string& drv8872_in2_gpio_line_name,
                   bool drv8872_in1_active_low, bool drv8872_in2_active_low);

    void forward(void);
    void backward(void);
    void coast(void);
    void brake(void);

private:
    /// @brief The GPIO line to motor in1 pin.
    std::unique_ptr<gpiod::line_request> motor_in1;
    /// @brief The GPIO line to motor in2 pin.
    std::unique_ptr<gpiod::line_request> motor_in2;
};
