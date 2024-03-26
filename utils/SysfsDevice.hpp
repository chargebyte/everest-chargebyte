#pragma once
#include <filesystem>
#include <string>

///
/// Helper class to access Linux kernel devices via sysfs filesystem.
/// The purpose of this class is to be a generic root for derived classes
/// to factor out common functions like reading or writing to property
/// files which resist for a given device below its root path.
///
class SysfsDevice {

public:
    /// @brief Default constructor.
    SysfsDevice(void);

    /// @brief Constructor.
    /// @param root A path pointing to the 'root' directory of the device
    ///             within the /sys hierarchy. This is usually an absolute path
    ///             but for testing purpose, this could also be a relative one.
    ///             If this directory does not exists, a `std::runtime` exception
    ///             is raised.
    SysfsDevice(const std::filesystem::path& root);

protected:
    /// @brief  Checks whether a given property (file) exists for this device.
    /// @param  name The property name (aka filename) to check.
    /// @return True, in case a file with this name exists, false if not.
    bool has_sys_attr(const std::string& name) const;

    /// @brief Write a given value into the given property file.
    /// @param name The property name (aka filename) to use.
    /// @param value The value to write.
    void set_sys_attr(const std::string& name, const std::string& value) const;

    /// @brief  Read the current value from the given property file.
    /// @param  name The property name (aka filename) of which the value should be read.
    /// @return The current value of the property.
    std::string get_sys_attr(const std::string& name) const;

    /// @brief Stores the root directory of the 'device' which is operated on.
    std::filesystem::path root;

};
