#pragma once
#include <string>
#include <gpiod.hpp>

///
/// Searches for a given, named GPIO line within all GPIO chips on the current platform, and if found
/// returns a preconfigured `gpiod::line_request` instance. If not found, throws a `std::runtime_error`.
/// The implementation iterates over all devices found in '/dev' directory.
///
/// @brief  Search for a named GPIO line to return a `gpiod::line_request` instance.
/// @param  name The name of the GPIO line which is searched for.
/// @param  consumer A string which is passed to the kernel as 'user' for this GPIO line.
/// @param  settings A `gpiod::line_settings` instance holding the desired settings when acquiring the GPIO.
/// @return A `gpiod::line_request` instance if the GPIO was found. Otherwise an exception is raised.
gpiod::line_request get_gpioline_by_name(const std::string& name, const std::string& consumer,
                                         const gpiod::line_settings& settings);

///
/// Searches for the given, named GPIO lines within all GPIO chips on the current platform, and if found
/// returns a preconfigured `gpiod::line_request` instance. If not found, throws a `std::runtime_error`.
/// The implementation iterates over all devices found in '/dev' directory.
/// Note: all elements are expected to be found in the same GPIO chip (aka bank).
///
/// @brief  Search for all named GPIO lines to return a `gpiod::line_request` instance.
/// @param  names A vector of names of the GPIO lines which are searched for.
/// @param  consumer A string which is passed to the kernel as 'user' for this GPIO line.
/// @param  settings A `gpiod::line_settings` instance holding the desired settings when acquiring the GPIO.
/// @return A `gpiod::line_request` instance if the GPIO was found. Otherwise an exception is raised.
gpiod::line_request get_gpiolines_by_name(const std::vector<std::string>& names, const std::string& consumer,
                                          const gpiod::line_settings& settings);
