#pragma once
#include <filesystem>
#include <string>
#include <vector>
#include "SysfsDevice.hpp"
#include "PWM.hpp"

///
/// A class for abstracting a Linux kernel PWM chip device represented in sysfs hierarchy.
/// The purpose of this class is to support iterating over the existing/available devices
/// to find a specific one by name. For this, the static helper functions can be used.
/// For unit testing purposes, there exist method variants which take an entry path, so that
/// this class can operate on a "faked" directory hierarchy.
///
class PWMChip : private SysfsDevice {

private:
    /// @brief Default sysfs path to operate on
    static const std::filesystem::path default_sysfs_path;

public:
    // Default constructor is deleted (RAII)
    PWMChip() = delete;

    /// @brief Constructor
    /// @param sysfs_path A path pointing to the 'root' directory of the device
    ///                   within the /sys hierarchy. This is usually an absolute path
    ///                   but for testing purpose, this could also be a relative one.
    ///                   If this directory does not exists, a `std::runtime` exception
    ///                   is raised.
    PWMChip(const std::filesystem::path& sysfs_path);

    /// @brief Read the name of the associated PWM chip device.
    /// @return The name of the PWM chip
    std::string get_name(void) const;

    /// @brief  Read the count of PWMs available in this PWM chip device.
    /// @return The count of PWMs.
    unsigned int get_pwm_count(void) const;

    /// @brief  Export a specific PWM in this PWM chip for later use.
    /// @param  number The number of the desired PWM.
    /// @return Returns an instance of class `PWM` for the desired PWM.
    PWM export_pwm(unsigned int number) const;

    /// @brief  Assumes that the given sysfs root path is holding PWM chip devices
    ///         and returns a list with `PWMChip` instances for each directory entry.
    ///         Note, that there is no deeper check whether the returned items are really
    ///         PWM chip devices.
    /// @param  sysfs_path The root directory to operate on.
    /// @return A vector with `PWMChip` instances.
    static std::vector<PWMChip> get_system_pwmchips(const std::filesystem::path& sysfs_path);

    /// @brief  Similar to `get_system_pwmchips(const std::filesystem::path sysfs_path)`
    ///         but operates on the default sysfs entry path '/sys/class/pwm'.
    /// @return A vector with `PWMChip` instances.
    static std::vector<PWMChip> get_system_pwmchips(void);

    /// @brief  Search for a PWM chip device with specific name. It uses `get_system_pwmchips`
    ///         and passes the given `sysfs_path` to it. If a matching PWM chip was found,
    ///         it returns a `PWMChip` instance for this device, if no match was found, a
    ///         `std::runtime_error` exception is raised.
    /// @param  name The name of the desired PWM chip device.
    /// @param  sysfs_path The root directory to operate on.
    /// @return A `PWMChip` instance if the desired device was found.
    static PWMChip find_pwmchip(const std::string& name, const std::filesystem::path& sysfs_path);

    /// @brief  Similar to `find_pwmchip(const std::string name, const std::filesystem::path sysfs_path)`
    ///         but operates on the default sysfs entry path '/sys/class/pwm'.
    /// @param  name The name of the desired PWM chip device.
    /// @return A `PWMChip` instance if the desired device was found.
    static PWMChip find_pwmchip(const std::string& name);

    /// @brief  Helper to find a specific PWM chip by name and export the given PWM.
    ///         Operates on the default sysfs entry path '/sys/class/pwm'.
    ///         It returns a `PWM` instance for the desired PWM device which is already
    ///         exported and thus ready to use.
    ///         If no match was found, a `std::runtime_error` exception is raised.
    /// @param  name The name of the PWM chip device.
    /// @param  number The number of the desired PWM.
    /// @return An `PWM` instance on success.
    static PWM find_pwm(const std::string& name, unsigned int number);

};
