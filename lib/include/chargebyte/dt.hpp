// Copyright © 2026 chargebyte GmbH
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <initializer_list>
#include <string>
#include <string_view>
#include <vector>

namespace chargebyte::dt {

/// @brief  Reads the DT compatible strings from current device.
/// @return A vector containing all compatible strings.
std::vector<std::string> read_dt_compatibles();

/// @brief  Checks whether a given compatible list includes a given compatible string.
/// @param  dt_compatibles List of DT compatibles as returned by `read_dt_compatibles`.
/// @param  value Compatible string to check whether it is included.
/// @return True if the given string is included, false otherwise.
bool is_dt_compatible(const std::vector<std::string>& dt_compatibles, const std::string& value);

/// @brief  Checks whether a given compatible list includes at least one of the items
///         given in the list of compatible strings.
/// @param  dt_compatibles List of DT compatibles as returned by `read_dt_compatibles`.
/// @param  values List of compatible strings to check whether one of these is included.
/// @return True if at least one of the given strings is included, false otherwise.
bool is_dt_compatible(const std::vector<std::string>& dt_compatibles, std::initializer_list<std::string_view> values);

} // namespace chargebyte::dt
