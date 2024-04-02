#pragma once
#include <string>
#include "IIOADC.hpp"

///
/// Search for an IIO ADC device with a given name. If found returns an IIOADC instance.
/// If not found, throws a `std::runtime_error`.
/// The implementation iterates over all devices found in '/sys/bus/iio/devices' directory.
///
/// @brief  Search for an IIO ADC device to return a `IIOADC` instance.
/// @param  name The name of the GPIO line which is searched for.
/// @return An `IIOADC` instance if the IIO ADC was found. Otherwise an exception is raised.
IIOADC get_iioadc_by_name(const std::string& name);
