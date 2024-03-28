// Copyright chargebyte GmbH
#pragma once
#include <chrono>
#include <PWM.hpp>
#include <PWMChip.hpp>
#include <string>

namespace module {

class CbTarragonDIPWM {
public:
    CbTarragonDIPWM() = delete;
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
    PWM pwm;
    void configure_digital_input_pwm();
};

} // namespace module
