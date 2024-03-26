#include <fstream>
#include <filesystem>
#include <iostream>
#include "SysfsDevice.hpp"

SysfsDevice::SysfsDevice(void) {
}

SysfsDevice::SysfsDevice(const std::filesystem::path& root) : root(root) {
    if (!std::filesystem::is_directory(this->root))
        throw std::runtime_error("The directory '" + root.generic_string() + "' does not exists.");
}

bool SysfsDevice::has_sys_attr(const std::string& name) const {
    return std::filesystem::exists(this->root / name);
}

void SysfsDevice::set_sys_attr(const std::string& name, const std::string &value) const {
    std::ofstream file;

    file.open(this->root / name);
    file << value;
    file.close();
}

std::string SysfsDevice::get_sys_attr(const std::string& name) const {
    std::ifstream file;
    std::string rv;

    file.open(this->root / name);
    file >> rv;
    file.close();

    return rv;
}
