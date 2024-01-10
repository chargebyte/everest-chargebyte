#include <filesystem>
#include <stdexcept>
#include <string>
#include "IIOADC.hpp"

IIOADC get_iioadc_by_name(const std::string& name) {

    for (const auto& entry : std::filesystem::directory_iterator("/sys/bus/iio/devices/")) {
        IIOADC device(entry.path());

        if (device.get_device_name() == name)
            return device;
    }

    throw std::runtime_error("No IIO ADC with name '" + name + "' found.");
}
