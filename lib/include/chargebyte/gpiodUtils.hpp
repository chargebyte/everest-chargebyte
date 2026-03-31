// Copyright © 2026 chargebyte GmbH
// SPDX-License-Identifier: Apache-2.0
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

///
/// Return the current/actual state of a given GPIO line. Uses `get_gpioline_by_name` internally, so
/// all restrictions and remarks are valid here, too.
///
/// @brief  Acquire the given GPIO, return its current value and release the GPIO.
/// @param  name The name of the GPIO line to inspect.
/// @param  active_low Whether the GPIO line should be threated with active-low semantic.
/// @param  consumer A string which is passed to the kernel as 'user' for this GPIO line.
/// @return True, whether the GPIO line is/was active, false otherwise.
bool get_gpioline_state_by_name(const std::string& name, bool active_low, const std::string& consumer);
