// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once
#include <string>
#include <memory>
#include <gpiod.hpp>
#include <PWM.hpp>

///
/// A class for abstracting a PWM device for Control Pilot on Tarragon platform.
/// On Tarragon platform, the PWM generation circuit uses an additional GPIO (CP_INVERT)
/// to allow disabling the pin output completely so that CP state E can be generated.
/// The underlying standard PWM is parametrized with the 1 kHz frequency as demanded by
/// the IEC 61851 standard.
/// Note that, disabling in this context really means that the pin output is not driven
/// anymore (CP state E). State F can be realized by setting a duty cycle of 0 %.
///
class CbTarragonPWM {

public:
    /// @brief Default constructor.
    CbTarragonPWM(void);

    /// @brief Constructor. The underlying PWM is disabled first, then re-configured
    ///        but left disabled. The pin output is disabled.
    ///        A specific PWM duty cycle and/or driving the signal must be later
    ///        achieved by using `set_duty_cycle`.
    /// @param pwm_device The name of the underlying PWM chip device.
    /// @param pwm_device_channel The number of the PWM within the PWM chip device.
    /// @param invert_gpioline_name The GPIO line name of the CP_INVERT signal.
    CbTarragonPWM(const std::string& pwm_device, unsigned int pwm_device_channel,
                  const std::string& invert_gpioline_name);

    /// @brief  Get the current duty cycle in percent.
    /// @return The current duty cycle.
    double get_duty_cycle(void) const;

    /// @brief Set a new duty cycle. Also enable the PWM and the CP_INVERT signal
    ///        if necessary so that the desired PWM signal actually shows up at the output pin.
    /// @param duty_cycle The desired duty cycle in percent.
    void set_duty_cycle(double duty_cycle);

    /// @brief Check whether the current duty cycle is nominal.
    /// @return True when configured duty cycle is >0 and <100% (nominal dutry cyle),
    ///         false otherwise.
    bool is_nominal_duty_cycle() const;

    /// @brief  Check whether the PWM output is actively driven.
    /// @return True when a signal is driven, false otherwise.
    bool is_enabled(void);

    /// @brief Disable the pin output. This drives 100% for a short time to ensure a
    ///        deterministic falling edge on the signal. Then the PWM is disabled and
    ///        CP_INVERT pin is deasserted to stop driving the pin output.
    void disable(void);

private:
    /// @brief The used PWM device
    PWM pwm;

    /// @brief The GPIO handle of the used CP_INVERT signal.
    std::unique_ptr<gpiod::line_request> cp_invert;

    /// @brief Helper to get the current CP_INVERT line state.
    bool get_cp_invert(void) const;

    /// @brief Helper to set the CP_INVERT line state.
    void set_cp_invert(bool active);
};
