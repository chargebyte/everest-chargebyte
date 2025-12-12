// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once
#include <string>
#include <mutex>
#include <cstddef>
#include <memory>
#include <vector>
#include <utility>
#include <cstddef>
#include <gpiod.hpp>

///
/// This class abstracts a GPIO line used to control the coil of a relais (and then
/// finally a contactor coil). Usually, it is constructed using exactly one GPIO line name,
/// but it is also possible to pass more line names. These are then expected to exist in
/// the same GPIO bank and are controlled simultanously when the first line passed is switched.
///
class CbActuator {

public:
    /// @brief Constructor
    /// @param name A string used as consumer in libgpiod.
    /// @param gpio_line_names Vector of GPIO line names to use.
    CbActuator(const std::string& name, const std::vector<std::string>& gpio_line_names);

    /// @brief Destructor.
    ~CbActuator();

    /// @brief Set the desired output value on actuator GPIO lines.
    ///        Note that the actual switching is done only when idx == 0 is updated.
    ///        It is assumed/required that all other index were set to the desired
    ///        output value before.
    /// @param on The target state (true = on, false = off)
    /// @param idx Index within the list of actuator GPIOs.
    void set_value(bool on, std::size_t idx);

    /// @brief Read the actual GPIO state of the given actuator index.
    bool get_value(std::size_t idx) const;

private:
    /// @brief The GPIO line request handle.
    std::unique_ptr<gpiod::line_request> line_req;

    /// @brief This vector holds the latest target values.
    gpiod::line::values values;
};

///
/// This is handy for users which receive a `CbActuator` pointer and then only want
/// to control a single line of a `CbActuator` with more than one GPIO lines.
/// Note: the second number is the index within the vector of GPIO line names,
/// not the actual offset within the GPIO bank.
///
using CbActuatorReference = std::pair<std::shared_ptr<CbActuator>, std::size_t>;
