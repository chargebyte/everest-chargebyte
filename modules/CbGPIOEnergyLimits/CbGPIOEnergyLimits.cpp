// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH, Pionix GmbH and Contributors to EVerest
#include "CbGPIOEnergyLimits.hpp"
#include <gpiodUtils.hpp>
#include <chrono>

using namespace std::chrono_literals;

namespace module {

void CbGPIOEnergyLimits::init() {
    std::vector<GpioLine> gpio_lines;

    // read DT compatibles
    this->init_dt_compatibles_list();

    if (this->is_dt_compatible({"chargebyte,imx6ull-tarragon-master", "chargebyte,imx6ull-tarragon-slave",
                                "chargebyte,imx6ull-tarragon-slavext", "chargebyte,imx6ull-tarragon-micro"})) {
        // current limit, phase count
        this->limits = {
            {6.0f, 1}, {10.0f, 1}, {13.0f, 1}, {16.0f, 1}, {20.0f, 1}, {32.0f, 1}, {40.0f, 1}, {63.0f, 1},
            {6.0f, 3}, {10.0f, 3}, {13.0f, 3}, {16.0f, 3}, {20.0f, 3}, {32.0f, 3}, {40.0f, 3}, {63.0f, 3},
        };

        gpio_lines = {
            {"ROTARY_SWITCH_1_1_N", true},
            {"ROTARY_SWITCH_1_2_N", true},
            {"ROTARY_SWITCH_1_4_N", true},
            {"ROTARY_SWITCH_1_8_N", true},
        };
    }

    if (this->is_dt_compatible("chargebyte,imx93-charge-som")) {
        // customer has no phase count limit per user manual, only current
        this->limits = {
            // first items are not covered by user manual
            {6.0f, std::nullopt},
            {6.0f, std::nullopt},
            {6.0f, std::nullopt},
            {6.0f, std::nullopt},
            {6.0f, std::nullopt},
            {6.0f, std::nullopt},
            {6.0f, std::nullopt},
            {6.0f, std::nullopt},
            // only the below ones are documented
            {6.0f, std::nullopt},
            {7.0f, std::nullopt},
            {8.0f, std::nullopt},
            {9.0f, std::nullopt},
            {10.0f, std::nullopt},
            {12.0f, std::nullopt},
            {13.0f, std::nullopt},
            {16.0f, std::nullopt},
        };

        gpio_lines = {
            {"CURRENT_SETTING_4", false},
            {"CURRENT_SETTING_3", false},
            {"CURRENT_SETTING_2", false},
            {"CURRENT_SETTING_1", false},
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

    std::size_t idx = this->get_gpios();

    if (idx > this->limits.size() - 1) {
        EVLOG_error << "Determined index of limits table out-of-range!";
        return;
    }

    auto [current_limit_A, max_phase_count] = this->limits[idx];

    EVLOG_info << "Limiting to " << current_limit_A << " A/phase, "
               << (max_phase_count.has_value()
                       ? (std::string("w/ max phase count of ") + std::to_string(max_phase_count.value()))
                       : "w/o max phase count");

    this->apply_new_limit(current_limit_A, max_phase_count);
}

void CbGPIOEnergyLimits::init_dt_compatibles_list() {
    // the compatible file is list of C strings, separated by NUL bytes

    std::string path = "/proc/device-tree/compatible";
    std::ifstream f(path, std::ios::binary);
    if (!f)
        throw std::runtime_error("Failed to open '" + path + "'");

    std::vector<char> buf((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

    std::size_t start = 0;
    while (start < buf.size()) {
        std::size_t end = start;
        while (end < buf.size() && buf[end] != '\0')
            ++end;

        if (end > start) {
            this->dt_compatibles.emplace_back(&buf[start], end - start);
        }

        start = end + 1;
    }
}

bool CbGPIOEnergyLimits::is_dt_compatible(const std::string& value) {
    return std::find(this->dt_compatibles.begin(), this->dt_compatibles.end(), value) != this->dt_compatibles.end();
}

bool CbGPIOEnergyLimits::is_dt_compatible(std::initializer_list<std::string_view> values) {

    for (std::string_view s : values) {
        if (std::find(dt_compatibles.begin(), dt_compatibles.end(), s) != dt_compatibles.end()) {
            return true;
        }
    }

    return false;
}

GpioLineRequest CbGPIOEnergyLimits::acquire_gpio(const GpioLine& line) {
    auto [gpio_line_name, active_low] = line;

    gpiod::line_settings line_settings;
    line_settings.set_direction(gpiod::line::direction::INPUT);
    line_settings.set_edge_detection(gpiod::line::edge::BOTH);
    line_settings.set_active_low(active_low);

    // a hard-coded value should do for here
    std::chrono::microseconds debounce_ms {25ms};
    line_settings.set_debounce_period(debounce_ms);

    return std::make_unique<gpiod::line_request>(
        get_gpioline_by_name(gpio_line_name, "CbGPIOEnergyLimits", line_settings));
}

void CbGPIOEnergyLimits::apply_new_limit(const float current_limit_A, std::optional<int> max_phase_count) {
    auto ts = Everest::Date::to_rfc3339(date::utc_clock::now());
    types::energy::ExternalLimits limits;

    limits.schedule_import.emplace_back(types::energy::ScheduleReqEntry {
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
        r->call_set_external_limits(limits);
    }
}

} // namespace module
