// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include <map>
#include <memory>
#include <utility>
#include "CbTarragonDriver.hpp"
#include "configuration.h"

namespace module {

void CbTarragonDriver::init() {
    EVLOG_info << MODULE_DESCRIPTION << " (version: " << MODULE_VERSION << ")";

    this->relays[this->config.relay_1_name] = std::make_unique<CbTarragonRelay>(
        this->config.relay_1_name, this->config.relay_1_actuator_gpio_line_name,
        this->config.relay_1_feedback_gpio_line_name, this->config.relay_1_feedback_gpio_debounce_us);

    this->relays[this->config.relay_2_name] = std::make_unique<CbTarragonRelay>(
        this->config.relay_2_name, this->config.relay_2_actuator_gpio_line_name,
        this->config.relay_2_feedback_gpio_line_name, this->config.relay_2_feedback_gpio_debounce_us);

    invoke_init(*p_evse_board_support);
    invoke_init(*p_ac_rcd);
}

void CbTarragonDriver::ready() {
    invoke_ready(*p_evse_board_support);
    invoke_ready(*p_ac_rcd);
}

} // namespace module
