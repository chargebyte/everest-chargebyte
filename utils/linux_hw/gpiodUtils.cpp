#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <gpiod.hpp>
#include "gpiodUtils.hpp"

gpiod::line_request get_gpioline_by_name(const std::string& name, const std::string& consumer,
                                         const gpiod::line_settings& settings) {

    for (const auto& entry : std::filesystem::directory_iterator("/dev/")) {
        if (gpiod::is_gpiochip_device(entry.path())) {
            gpiod::chip chip(entry.path());

            auto offset = chip.get_line_offset_from_name(name);
            if (offset >= 0) {
                // clang-format off
                return chip
                    .prepare_request()
                    .set_consumer(consumer)
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
                    .set_consumer(consumer)
                    .add_line_settings(offsets, settings)
                    .do_request();
                // clang-format on
            }
        }
    }

    throw std::runtime_error("No GPIO bank found which provides all GPIO lines (" +
                             join_into_string_with_quotes(names) + ").");
}
