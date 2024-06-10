// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once
#include <chrono>
#include <PWM.hpp>
#include <PWMChip.hpp>
#include <string>

namespace module {

///
/// This class abstracts the control of the reference PWM required for digital inputs.
///
class CbTarragonDIPWM {
public:
    CbTarragonDIPWM() = delete;
    /// @brief Constructor.
    /// @param device The unique name of the PWM device to use as digital input reference.
    /// @param channel The PWM channel number to use as digital input reference.
    /// @param threshold The threshold voltage for the digital inputs in mV. Values above this
    ///                  threshold are considered as high.
    CbTarragonDIPWM(const std::string& device, int channel, int threshold) :
        device(device), channel(channel), threshold_voltage(threshold), pwm(PWMChip::find_pwm(device, channel)) {
        configure_digital_input_pwm();
    };

protected:
private:
    static constexpr std::chrono::nanoseconds PWM_PERIOD {40000}; // equals 25 kHz
    static constexpr int MAX_THRESHOLD_VOLTAGE {12000};           // mV

    std::string device;
    int channel {0};
    int threshold_voltage {0};
    /// @brief The PWM object
    PWM pwm;
    /// @brief Configure the digital input PWM
    void configure_digital_input_pwm();
};

} // namespace module
