// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include <chrono>
#include <memory>
#include <ostream>
#include <string>
#include <sigslot/signal.hpp>
#include <generated/types/cb_board_support.hpp>
#include "CbTarragonRelay.hpp"
#include "CbTarragonContactor.hpp"

CbTarragonContactor::CbTarragonContactor(const std::string& name, std::unique_ptr<CbTarragonRelay> relay,
                                         const std::string& contactor_feedback_type) :
    name(name),
    feedback_type(types::cb_board_support::string_to_contactor_feedback_type(contactor_feedback_type)),
    relay(std::move(relay)) {
    // register error handler
    this->relay->on_unexpected_change.connect([&](const std::string& relay_name, bool seen_state) {
        types::cb_board_support::ContactorState seen_contactor_state {
            seen_state ? types::cb_board_support::ContactorState::Closed
                       : types::cb_board_support::ContactorState::Open};

        this->on_unexpected_change(this->name + "@" + relay_name, seen_contactor_state);
    });

    // start the monitoring of the feedback contact (if desired);
    // when the wired feedback is normally-closed type, then we see a physical HIGH on underlying
    // GPIO when the contactor is considered open, thus we need to invert the used logic
    this->relay->start(this->feedback_type != types::cb_board_support::ContactorFeedbackType::none,
                       this->feedback_type == types::cb_board_support::ContactorFeedbackType::nc);
}

const std::string& CbTarragonContactor::get_name() const {
    return this->name;
}

bool CbTarragonContactor::switch_state(bool on, bool wait_for_feedback) {
    return this->relay->set_actuator_state(on, wait_for_feedback);
}

bool CbTarragonContactor::get_state() const {
    return this->relay->get_actuator_state();
}

types::cb_board_support::ContactorState CbTarragonContactor::get_feedback_state() const {
    return this->relay->get_feedback_state() ? types::cb_board_support::ContactorState::Closed
                                             : types::cb_board_support::ContactorState::Open;
}

bool CbTarragonContactor::is_state_mismatch() const {
    return this->relay->get_actuator_state() != this->relay->get_feedback_state();
}

void CbTarragonContactor::set_expected_feedback_change(bool on) {
    this->relay->set_expected_feedback_change(on);
}

bool CbTarragonContactor::wait_for_feedback() {
    return this->relay->wait_for_feedback();
}

std::chrono::milliseconds CbTarragonContactor::get_closing_delay_left() const {
    return this->relay->get_closing_delay_left();
}

std::ostream& operator<<(std::ostream& os, const CbTarragonContactor& c) {
    os << c.name << "@" << *c.relay;
    return os;
}
