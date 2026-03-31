// Copyright © 2026 chargebyte GmbH
// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <gpiod.hpp>
#include <chargebyte/gpiodUtils.hpp>

using namespace std::chrono_literals;

gpiod::line_request get_gpioline_by_name(const std::string& name, const std::string& consumer,
                                         const gpiod::line_settings& settings) {
    std::string filtered_consumer = consumer;

    // check if the actuator name contains the character '/' and find its position. This is done because the
    // kernel throws a warning when a GPIO consumer has '/' in its name.
    size_t pos = filtered_consumer.find('/');

    // if '/' is found, replace it with '-'
    if (pos != std::string::npos)
        filtered_consumer.replace(pos, 1, "-");

    for (const auto& entry : std::filesystem::directory_iterator("/dev/")) {
        if (gpiod::is_gpiochip_device(entry.path())) {
            gpiod::chip chip(entry.path());

            auto offset = chip.get_line_offset_from_name(name);
            if (offset >= 0) {
                // clang-format off
                return chip
                    .prepare_request()
                    .set_consumer(filtered_consumer)
                    .add_line_settings(offset, settings)
                    .do_request();
                // clang-format on
            }
        }
    }

    throw std::runtime_error("No GPIO line with name '" + name + "' found.");
}

static std::string join_into_string_with_quotes(const std::vector<std::string>& items) {
    std::ostringstream oss;
    bool first = true;
    for (const auto& item : items) {
        if (!first)
            oss << ", ";
        first = false;
        oss << '\'' << item << '\''; // add quotes around each item
    }
    return oss.str();
}

gpiod::line_request get_gpiolines_by_name(const std::vector<std::string>& names, const std::string& consumer,
                                          const gpiod::line_settings& settings) {
    std::string filtered_consumer = consumer;

    // check if the actuator name contains the character '/' and find its position. This is done because the
    // kernel throws a warning when a GPIO consumer has '/' in its name.
    size_t pos = filtered_consumer.find('/');

    // if '/' is found, replace it with '-'
    if (pos != std::string::npos)
        filtered_consumer.replace(pos, 1, "-");

    for (const auto& entry : std::filesystem::directory_iterator("/dev/")) {
        if (gpiod::is_gpiochip_device(entry.path())) {
            gpiod::chip chip(entry.path());

            gpiod::line::offsets offsets;

            for (const auto& name : names) {
                auto offset = chip.get_line_offset_from_name(name);
                if (offset >= 0) {
                    offsets.push_back(offset);
                }
            }

            // check whether all names are found within this chip
            if (names.size() == offsets.size()) {
                // clang-format off
                return chip
                    .prepare_request()
                    .set_consumer(filtered_consumer)
                    .add_line_settings(offsets, settings)
                    .do_request();
                // clang-format on
            }
        }
    }

    throw std::runtime_error("No GPIO bank found which provides all GPIO lines (" +
                             join_into_string_with_quotes(names) + ").");
}

bool get_gpioline_state_by_name(const std::string& name, bool active_low, const std::string& consumer) {

    gpiod::line_settings line_settings;
    line_settings.set_direction(gpiod::line::direction::INPUT);
    line_settings.set_active_low(active_low);

    // a hard-coded value should do for here
    std::chrono::microseconds debounce_ms{50ms};
    line_settings.set_debounce_period(debounce_ms);

    auto line_request = get_gpioline_by_name(name, consumer, line_settings);

    bool rv = line_request.get_value(line_request.offsets()[0]) == gpiod::line::value::ACTIVE;

    line_request.release();

    return rv;
}
