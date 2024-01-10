#pragma once
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include "SysfsDevice.hpp"

///
/// A class for abstracting a Linux kernel IIO ADC device represented in sysfs hierarchy.
/// The constructor is given the root directory of the ADC device.
/// When constructed, the class reads the name property of the device.
/// The restriction of this class is that only a single channel can be managed.
///
class IIOADC : private SysfsDevice {

public:
    /// @brief Default constructor.
    IIOADC(void);

    /// @brief Constructor.
    /// @param sysfs_path A path pointing to the 'root' directory of the IIO ADC device
    ///                   within the /sys hierarchy, e.g. '/sys/bus/iio/devices/iio:device0'
    IIOADC(const std::filesystem::path& sysfs_path);

    /// @brief  Get the name of the associated device.
    /// @return The name of the device.
    std::string get_device_name(void) const;

    /// @brief  Check whether the device has a given channel.
    /// @param  channel_name The name of the requested channel, e.g. 'voltage5'.
    /// @return True in case the channel exists, false otherwise.
    bool has_channel(const std::string& channel_name) const;

    /// @brief Open the given channel. Raises an exception when called more
    ///        than once, or opening failed.
    /// @param channel_name The name of the requested channel, e.g. 'voltage5'.
    void open_channel(const std::string& channel_name);

    /// @brief Read the current value from the opened channel.
    /// @return The value as found in 'in_<channel_name>_raw'.
    long int get_value(void);

private:
    /// @brief Cached device name
    std::string device_name;

    /// @brief Input stream of the requested channel
    std::ifstream channel;

    /// @brief Return the filename of an channel
    std::string get_channel_filename(const std::string& channel_name) const;

};
