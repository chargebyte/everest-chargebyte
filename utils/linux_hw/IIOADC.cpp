#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include "SysfsDevice.hpp"
#include "IIOADC.hpp"

IIOADC::IIOADC(void) {
}

IIOADC::IIOADC(const std::filesystem::path& sysfs_path) : SysfsDevice(sysfs_path), device_name("unknown") {
    if (this->has_sys_attr("name"))
        this->device_name = this->get_sys_attr("name");
}

std::string IIOADC::get_device_name(void) const {
    return this->device_name;
}

bool IIOADC::has_channel(const std::string& channel_name) const {
    std::string rel_filename = this->get_channel_filename(channel_name);

    return this->has_sys_attr(rel_filename);
}

void IIOADC::open_channel(const std::string& channel_name) {
    std::string rel_filename = this->get_channel_filename(channel_name);

    if (this->channel.is_open())
        throw std::runtime_error("A channel was already associated/opened with this object.");

    this->channel.open(this->root / rel_filename);

    if (!this->channel.is_open())
        throw std::runtime_error("Error opening the requested channel.");
}

long int IIOADC::get_value(void) {
    long int rv;

    if (!this->channel.is_open())
        throw std::runtime_error("A specific channel was not opened yet via 'open_channel'.");

    // rewind to start of file
    this->channel.seekg(0);

    // read current value
    this->channel >> rv;

    return rv;
}

std::string IIOADC::get_channel_filename(const std::string& channel_name) const {
    return "in_" + channel_name + "_raw";
}
