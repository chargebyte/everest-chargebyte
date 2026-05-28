// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH, Pionix GmbH and Contributors to EVerest
#include "CbGPIOEnergyLimits.hpp"
#include <poll.h>
#include <chrono>
#include <chargebyte/dt.hpp>
#include <chargebyte/gpiodUtils.hpp>

using namespace std::chrono_literals;
using namespace chargebyte::dt;

namespace module {

void CbGPIOEnergyLimits::init() {
    std::vector<GpioLine> gpio_lines;

    // the list of DT compatible strings of the board we are running on
    auto dt_compatibles = read_dt_compatibles();

    if (is_dt_compatible(dt_compatibles,
                         {"chargebyte,imx6ull-tarragon-master", "chargebyte,imx6ull-tarragon-slave",
                          "chargebyte,imx6ull-tarragon-slavext", "chargebyte,imx6ull-tarragon-micro"})) {
        // phase count limit is evaluated by BSP so not considered here
        this->limits = {
            {6.0f, std::nullopt},  {10.0f, std::nullopt}, {13.0f, std::nullopt}, {16.0f, std::nullopt},
            {20.0f, std::nullopt}, {32.0f, std::nullopt}, {40.0f, std::nullopt}, {63.0f, std::nullopt},
        };

        gpio_lines = {
            {"ROTARY_SWITCH_1_1_N", true},
            {"ROTARY_SWITCH_1_2_N", true},
            {"ROTARY_SWITCH_1_4_N", true},
        };
    }

    if (is_dt_compatible(dt_compatibles, "chargebyte,imx93-ac-power-board")) {
        // phase count limit is evaluated by BSP so not considered here
        this->limits = {
            {6.0f, std::nullopt},  {7.0f, std::nullopt},  {8.0f, std::nullopt},  {9.0f, std::nullopt},
            {10.0f, std::nullopt}, {12.0f, std::nullopt}, {13.0f, std::nullopt}, {16.0f, std::nullopt},
        };

        gpio_lines = {
            {"CURRENT_SETTING_4", false},
            {"CURRENT_SETTING_3", false},
            {"CURRENT_SETTING_2", false},
        };
    }

    // configure all given GPIOs
    for (const auto& line : gpio_lines) {
        this->gpios.push_back(this->acquire_gpio(line));
    }
}

std::size_t CbGPIOEnergyLimits::get_gpios() {
    std::size_t rv = 0;

    for (size_t i = 0; i < this->gpios.size(); ++i) {
        rv |= static_cast<std::size_t>(this->gpios[i]->get_value(this->gpios[i]->offsets()[0]) ==
                                       gpiod::line::value::ACTIVE)
              << i;
    }

    return rv;
}

void CbGPIOEnergyLimits::ready() {
    struct pollfd fds[this->gpios.size()];
    int rv;

    for (size_t i = 0; i < this->gpios.size(); ++i) {
        fds[i].fd = this->gpios[i]->fd();
        fds[i].events = POLLIN;
    }

    do {
        // read the current GPIO state
        std::size_t idx = this->get_gpios();

        if (idx < this->limits.size()) {
            auto [current_limit_A, max_phase_count] = this->limits[idx];

            EVLOG_info << "Limiting to " << current_limit_A << " A/phase, "
                       << (max_phase_count.has_value()
                               ? (std::string("w/ max phase count of ") + std::to_string(max_phase_count.value()))
                               : "w/o max phase count");

            this->apply_new_limit(current_limit_A, max_phase_count);
        } else {
            EVLOG_error << "Determined index of limits table is out of range! Ignored.";
        }

        // sleep until any of the GPIO lines reported an event
        rv = poll(fds, this->gpios.size(), -1);
        if (rv > 0) {
            // we don't really care about the actual events since we just loop and
            // read the actual GPIO state again in the loop above,
            // but we need to flush the events
            for (size_t i = 0; i < this->gpios.size(); ++i) {
                if (fds[i].revents & POLLIN) {
                    gpiod::edge_event_buffer buf;
                    this->gpios[i]->read_edge_events(buf);
                }
            }
        }
    } while (rv >= 0);
}

GpioLineRequest CbGPIOEnergyLimits::acquire_gpio(const GpioLine& line) {
    auto [gpio_line_name, active_low] = line;

    gpiod::line_settings line_settings;
    line_settings.set_direction(gpiod::line::direction::INPUT);
    line_settings.set_edge_detection(gpiod::line::edge::BOTH);
    line_settings.set_active_low(active_low);

    // a hard-coded value should do for here
    std::chrono::microseconds debounce_ms {50ms};
    line_settings.set_debounce_period(debounce_ms);

    return std::make_unique<gpiod::line_request>(
        get_gpioline_by_name(gpio_line_name, "CbGPIOEnergyLimits", line_settings));
}

void CbGPIOEnergyLimits::apply_new_limit(const float current_limit_A, std::optional<int> max_phase_count) {
    auto ts = Everest::Date::to_rfc3339(date::utc_clock::now());
    types::energy::ExternalLimits new_external_limits;

    new_external_limits.schedule_import.emplace_back(types::energy::ScheduleReqEntry {
        // timestamp
        .timestamp = ts,

        // root facing side
        .limits_to_root =
            {
                .total_power_W = std::nullopt,
                .ac_max_current_A =
                    types::energy::NumberWithSource {.value = current_limit_A, .source = "CbGPIOEnergyLimits"},
                .ac_min_current_A = std::nullopt,
                .ac_max_phase_count =
                    max_phase_count.has_value()
                        ? std::optional<types::energy::IntegerWithSource> {types::energy::IntegerWithSource {
                              .value = max_phase_count.value(), .source = "CbGPIOEnergyLimits"}}
                        : std::nullopt,
                .ac_min_phase_count = std::nullopt,
                .ac_supports_changing_phases_during_charging = std::nullopt,
                .ac_number_of_active_phases = std::nullopt,
            },

        // leaves facing side
        .limits_to_leaves = {},

        // no conversion efficiency
        .conversion_efficiency = std::nullopt,

        // no price info
        .price_per_kwh = std::nullopt,
    });

    // push limits
    for (const auto& r : this->r_energy_sink) {
        r->call_set_external_limits(new_external_limits);
    }
}

} // namespace module
