#include <filesystem>
#include <string>
#include <gpiod.hpp>
#include "gpiodUtils.hpp"

gpiod::line_request get_gpioline_by_name(const std::string& name, const std::string& consumer, const gpiod::line_settings& settings) {

    for (const auto& entry : std::filesystem::directory_iterator("/dev/")) {
        if (gpiod::is_gpiochip_device(entry.path())) {
            gpiod::chip chip(entry.path());

            auto offset = chip.get_line_offset_from_name(name);
            if (offset >= 0) {
                return chip
                    .prepare_request()
                    .set_consumer(consumer)
                    .add_line_settings(offset, settings)
                    .do_request();
            }
        }
    }

    throw std::runtime_error("No GPIO line with name '" + name + "' found.");
}
