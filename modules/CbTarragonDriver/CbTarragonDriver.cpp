// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include <map>
#include <memory>
#include <utility>
#include "CbTarragonDriver.hpp"
#include <CbActuator.hpp>
#include <CbTarragonRelay.hpp>
#include <CbTarragonContactorControl.hpp>
#include <CbTarragonContactorControlSimple.hpp>
#include <CbTarragonContactorControlSerial.hpp>
#include <CbTarragonContactorControlSimultaneous.hpp>
#include <CbTarragonContactorControlMutual.hpp>
#include "configuration.h"

namespace module {

void CbTarragonDriver::init() {
    CbActuatorReference actuator_ref_1;
    CbActuatorReference actuator_ref_2;

    EVLOG_info << MODULE_DESCRIPTION << " (version: " << MODULE_VERSION << ")";

    // except for the special wiring "simultaneous", let's create two
    // independent actuator instances
    if (this->config.switch_3ph1ph_wiring == "simultaneous") {
        actuator_ref_1 =
            std::make_pair(std::make_shared<CbActuator>(
                               "CbActuator (" + this->config.relay_1_name + ", " + this->config.relay_2_name + ")",
                               std::vector<std::string> {this->config.relay_1_actuator_gpio_line_name,
                                                         this->config.relay_2_actuator_gpio_line_name}),
                           0);

        actuator_ref_2 = std::make_pair(actuator_ref_1.first, 1);
    } else {
        actuator_ref_1 = std::make_pair(
            std::make_shared<CbActuator>("CbActuator (" + this->config.relay_1_name + ")",
                                         std::vector<std::string> {this->config.relay_1_actuator_gpio_line_name}),
            0);

        actuator_ref_2 = std::make_pair(
            std::make_shared<CbActuator>("CbActuator (" + this->config.relay_2_name + ")",
                                         std::vector<std::string> {this->config.relay_2_actuator_gpio_line_name}),
            0);
    }

    this->relays[this->config.relay_1_name] = std::make_unique<CbTarragonRelay>(
        this->config.relay_1_name, actuator_ref_1, this->config.relay_1_feedback_gpio_line_name,
        this->config.relay_1_feedback_gpio_debounce_us);

    this->relays[this->config.relay_2_name] = std::make_unique<CbTarragonRelay>(
        this->config.relay_2_name, actuator_ref_2, this->config.relay_2_feedback_gpio_line_name,
        this->config.relay_2_feedback_gpio_debounce_us);

    // contactor instance based on configuration
    if (this->config.switch_3ph1ph_wiring == "none") {
        // per definition we use the relay 1 as primary/only relay
        auto nh_relay = this->relays.extract(this->config.relay_1_name);

        this->contactor_controller = std::make_unique<CbTarragonContactorControlSimple>(
            std::move(nh_relay.mapped()), this->config.contactor_1_feedback_type);

    } else if (this->config.switch_3ph1ph_wiring == "serial") {
        // per definition we use the relay 1 as primary relay (all phases)
        // and relay 2 as secondary relay (phases 2 and 3)
        auto nh_primary = this->relays.extract(this->config.relay_1_name);
        auto nh_secondary = this->relays.extract(this->config.relay_2_name);

        this->contactor_controller = std::make_unique<CbTarragonContactorControlSerial>(
            std::move(nh_primary.mapped()), this->config.contactor_1_feedback_type, std::move(nh_secondary.mapped()),
            this->config.contactor_2_feedback_type);

    } else if (this->config.switch_3ph1ph_wiring == "simultaneous") {
        // here we assume that primary switches one phase and neutral, the
        // secondary the remaining other phases
        auto nh_primary = this->relays.extract(this->config.relay_1_name);
        auto nh_secondary = this->relays.extract(this->config.relay_2_name);

        this->contactor_controller = std::make_unique<CbTarragonContactorControlSimultaneous>(
            std::move(nh_primary.mapped()), this->config.contactor_1_feedback_type, std::move(nh_secondary.mapped()),
            this->config.contactor_2_feedback_type);

    } else if (this->config.switch_3ph1ph_wiring == "mutual") {
        // per definition we use the relay 1 as primary relay (3ph) and relay 2 as secondary relay (1ph)
        auto nh_3ph = this->relays.extract(this->config.relay_1_name);
        auto nh_1ph = this->relays.extract(this->config.relay_2_name);

        this->contactor_controller = std::make_unique<CbTarragonContactorControlMutual>(
            std::move(nh_3ph.mapped()), this->config.contactor_1_feedback_type, std::move(nh_1ph.mapped()),
            this->config.contactor_2_feedback_type);
    }

    invoke_init(*p_evse_board_support);
    invoke_init(*p_ac_rcd);

    // now that the interface inits are done, we still need to ensure that RCM faults open the contactor
    if (this->config.rcm_enable) {
        this->rcm_controller.on_change.connect([&](bool rcm_tripped) {
            // remember to prevent closing again
            this->contactor_controller->is_emergency = rcm_tripped;

            if (rcm_tripped) {
                // open the contactor without waiting for EvseManager
                this->contactor_controller->open();

                // also publish PowerOff event
                types::board_support_common::Event tmp_event = types::board_support_common::Event::PowerOff;
                types::board_support_common::BspEvent tmp {tmp_event};
                this->p_evse_board_support->publish_event(tmp);

                EVLOG_warning << "RCM fault occurred";
            } else {
                EVLOG_info << "RCM fault released";
            }
        });
    }
}

void CbTarragonDriver::ready() {
    invoke_ready(*p_evse_board_support);
    invoke_ready(*p_ac_rcd);
}

} // namespace module
