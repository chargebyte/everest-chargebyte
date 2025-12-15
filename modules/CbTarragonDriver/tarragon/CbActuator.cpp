// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include <algorithm>
#include <memory>
#include <ostream>
#include <string>
#include <gpiod.hpp>
#include <gpiodUtils.hpp>
#include "CbActuator.hpp"

CbActuator::CbActuator(const std::string& name, const std::vector<std::string>& gpio_line_names) :
    values(gpio_line_names.size(), gpiod::line::value::INACTIVE) {

    // instantiate and configure the actuator gpio
    this->line_req =
        std::make_unique<gpiod::line_request>(get_gpiolines_by_name(gpio_line_names, name,
                                                                    gpiod::line_settings()
                                                                        .set_direction(gpiod::line::direction::OUTPUT)
                                                                        .set_output_value(gpiod::line::value::INACTIVE)
                                                                        .set_active_low(false)));
}

CbActuator::~CbActuator() {
    // ensure that all are reset to inactive
    std::fill(this->values.begin(), this->values.end(), gpiod::line::value::INACTIVE);

    this->line_req->set_values(this->values);
}

void CbActuator::set_value(bool on, std::size_t idx) {

    this->values[idx] = on ? gpiod::line::value::ACTIVE : gpiod::line::value::INACTIVE;

    // actually toggle only when first item is set
    if (idx == 0) {
        this->line_req->set_values(this->values);
    }
}

bool CbActuator::get_value(std::size_t idx) const {
    return this->line_req->get_value(this->line_req->offsets()[idx]) == gpiod::line::value::ACTIVE;
}
