// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include "CbTarragonRCM.hpp"
#include <gpiod.hpp>
#include <gpiodUtils.hpp>
#include <chrono>

CbTarragonRCM::CbTarragonRCM() {
}

CbTarragonRCM::CbTarragonRCM(const std::string& rcm_fault_gpio_line_name, const bool& rcm_fault_active_low) {

    this->rcm_fault =
        std::make_unique<gpiod::line_request>(get_gpioline_by_name(rcm_fault_gpio_line_name, "RCM",
                                                                   gpiod::line_settings()
                                                                       .set_direction(gpiod::line::direction::INPUT)
                                                                       .set_edge_detection(gpiod::line::edge::BOTH)
                                                                       .set_active_low(rcm_fault_active_low)));
}

bool CbTarragonRCM::is_rcm_tripped() const {
    return this->rcm_fault->get_value(this->rcm_fault->offsets()[0]) == gpiod::line::value::ACTIVE;
}

void CbTarragonRCM::wait_for_rcm_event(const std::chrono::nanoseconds& timeout) {
    // waits for a GPIO edge event with timeout
    this->rcm_fault->wait_edge_events(timeout);
}