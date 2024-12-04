#pragma once
#include <chrono>
#include <filesystem>
#include "SysfsDevice.hpp"

///
/// A class for abstracting a Linux kernel PWM device represented in sysfs hierarchy.
/// The constructor is given the root directory of the PWM device, which means that
/// the device must already be exported by the upper PWM chip device.
/// When constructed, the class reads all current properties from the sysfs and
/// caches these values in private members. It assumes that no other party operates
/// on the same device, otherwise the cached values would run out-of-sync.
/// While the 'period' and 'duty_cycle' stuff mirrors the Linux kernel's representation,
/// the 'ratio' uses a floating point value for the duty cycle between 0 and 1.
///
class PWM : private SysfsDevice {

public:
    /// @brief Default constructor.
    PWM(void);

    /// @brief Constructor.
    /// @param sysfs_path A path pointing to the 'root' directory of the PWM device
    ///                   within the /sys hierarchy, e.g. '/sys/class/pwm/pwmchip2/pwm0'
    PWM(const std::filesystem::path& sysfs_path);

    /// @brief Destructor. Disables the PWM if still enabled.
    ~PWM(void);

    /// @brief  Retrieve the currently configured period of the PWM.
    /// @return The (cached) PWM period.
    std::chrono::nanoseconds get_period(void) const;

    /// @brief Set the period of the PWM. If the period is smaller than the
    ///        current duty cycle, then the duty cycle is adjusted automatically.
    /// @param period The new period.
    void set_period(std::chrono::nanoseconds period);

    /// @brief  Retrieve the currently configured duty cycle of the PWM.
    /// @return The (cached) duty cycle.
    std::chrono::nanoseconds get_duty_cycle(void) const;

    /// @brief Set a new duty cycle. The duty cycle must be less then
    ///        the currently configured period, otherwise a `std::invalid_argument`
    ///        exception is raised.
    /// @param duty_cycle The new duty cycle.
    void set_duty_cycle(std::chrono::nanoseconds duty_cycle);

    /// @brief  Get the current duty cycle as ratio between 0 and 1.
    /// @return The (cached) ratio value.
    float get_ratio(void) const;

    /// @brief Set a new duty cycle, given as ration between 0 and 1.
    /// @param ratio The new ratio.
    void set_ratio(float ratio);

    /// @brief  Retrieve the current (cached) state of the PWM device.
    /// @return Return whether the PWM is active or not.
    bool is_enabled(void) const;

    /// @brief Enable or disable the PWM device.
    /// @param enable True for enabling the device, false for disabling it.
    void set_enabled(bool enable);

    /// @brief  Check whether the PWM device operates in inversed mode.
    /// @return True if inversed mode is enabled, false otherwise (normal mode).
    bool is_inverted(void) const;

    /// @brief Set the PWM device to normal or inversed output mode.
    /// @param invert True for inversed/inverted mode, false for normal mode.
    void set_inverted(bool invert);

    /// @brief Configure the PWM parameters using a frequency and ratio.
    ///        The current `enabled` state of the PWM is left alone.
    /// @param frequency The frequency of the PWM.
    /// @param ratio The new ratio of the PWM.
    void set_frequency_and_ratio(unsigned long long frequency, float ratio);

private:
    /// @brief Cached duty cycle
    std::chrono::nanoseconds duty_cycle;

    /// @brief Cached period
    std::chrono::nanoseconds period;

    /// @brief Cached 'enable' flag
    bool enabled;

    /// @brief Cache 'inverted' flag
    bool inverted;

    /// @brief Used by the constructor to initialize/read all current properties.
    void read_current_values(void);

};
