// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include <chrono>
#include <memory>
#include <mutex>
#include <ostream>
#include <optional>
#include <string>
#include <thread>
#include <everest/logging.hpp>
#include <sigslot/signal.hpp>
#include <generated/types/cb_board_support.hpp>
#include "CbCarrierBoardRelay.hpp"

using namespace std::chrono_literals;

CbCarrierBoardRelay::CbCarrierBoardRelay(CbChargeSOM& controller, std::initializer_list<Contactor> contactors,
                                         const std::string& relay_name, bool enslaved) :
    controller(controller), relay_name(relay_name), is_enslaved(enslaved) {
    // convert the used relays into their offsets
    for (const auto contactor : contactors) {
        this->contactor_indexes.insert(static_cast<unsigned int>(contactor));
    }
}

CbCarrierBoardRelay::~CbCarrierBoardRelay() {
    // safety: explicitly open the relay, but don't wait for any feedback
    this->set_actuator_state(false, false);
}

void CbCarrierBoardRelay::start(bool use_feedback, bool active_low) {
    (void)active_low; // not used here: configured in safety controller

    this->has_feedback = use_feedback;

    if (use_feedback) {
        this->controller.on_contactor_change.connect(
            [&](const unsigned int idx, types::cb_board_support::ContactorState actual_state) {
                // check whether the index is of interest at all (we are called for all contactors)
                if (!this->contactor_indexes.count(idx))
                    return;

                EVLOG_debug << "Contactor " << (idx + 1) << " state change detected: now " << actual_state;

                if (actual_state == types::cb_board_support::ContactorState::Unknown) {
                    return;
                }

                // check whether the event was expected...
                if (this->expected_edge.has_value() and !this->expected_edge_matched.has_value()) {
                    // ...and whether the actual edge matches our expectation
                    this->expected_edge_matched = actual_state == this->expected_edge.value();

                    // signal the result to `set_actuator_state`
                    this->cv_expected_edge.notify_one();

                    // clear the expected edge
                    this->expected_edge.reset();
                }

                // report also via on_change event
                this->on_change(this->relay_name, actual_state == types::cb_board_support::ContactorState::Closed);
            });

        this->controller.on_contactor_error.connect(
            [&](const unsigned int idx, bool target_state, types::cb_board_support::ContactorState actual_state) {
                // check whether the index is of interest at all (we are called for all contactors)
                if (!this->contactor_indexes.count(idx))
                    return;

                EVLOG_debug << "Contactor " << (idx + 1) << " error detected: expected "
                            << (target_state ? "CLOSED" : "OPEN") << ", is " << actual_state;
            });
    }
}

bool CbCarrierBoardRelay::wait_for_feedback_lock(std::unique_lock<std::mutex>& lock) {
    // in case we have no feedback to verify our operation, or we don't expect a state change,
    // then just return success
    if (!this->has_feedback or !this->expected_edge.has_value())
        return true;

    // sleep until the edge was seen or a timeout occurred
    this->cv_expected_edge.wait_for(lock, this->feedback_timeout,
                                    [&] { return this->expected_edge_matched.has_value(); });

    // take over the result (or false on timeout)
    return this->expected_edge_matched.value_or(false);
}

bool CbCarrierBoardRelay::wait_for_feedback() {
    std::unique_lock<std::mutex> lock(this->expected_edge_mutex);

    return this->wait_for_feedback_lock(lock);
}

bool CbCarrierBoardRelay::set_actuator_state(bool on, bool wait_for_feedback) {
    std::unique_lock<std::mutex> lock(this->expected_edge_mutex);

    // check whether we actually need to toggle
    if (on == this->get_actuator_state())
        return true;

    // determine whether we are expecting a state change on feedback
    this->set_expected_edge(on);

    // actually toggle the relay(s)
    for (const auto& idx : this->contactor_indexes) {
        this->controller.set_contactorN_state(idx, on);
    }
    if (!this->is_enslaved) {
        this->controller.set_contactorN_state_commit();
    }

    // in case we don't want to wait, then just return success
    if (!wait_for_feedback)
        return true;

    return this->wait_for_feedback_lock(lock);
}

void CbCarrierBoardRelay::set_expected_edge(bool on) {
    // if we have a feedback, let's check the current state to determine, whether we can expect a state change
    if (this->has_feedback and this->get_feedback_state() == this->get_actuator_state()) {
        // remember what to expect next
        this->expected_edge =
            on ? types::cb_board_support::ContactorState::Closed : types::cb_board_support::ContactorState::Open;
    } else {
        // no state change expected
        this->expected_edge.reset();
    }

    // reset any left over return value
    this->expected_edge_matched.reset();
}

bool CbCarrierBoardRelay::get_actuator_state() const {
    bool rv = false;

    for (const auto& idx : this->contactor_indexes) {
        rv |= this->controller.get_contactorN_target_state(idx);
    }

    return rv;
}

bool CbCarrierBoardRelay::get_feedback_state() const {
    bool rv = false;

    if (!this->has_feedback)
        return this->get_actuator_state();

    for (const auto& idx : this->contactor_indexes) {
        rv |= this->controller.get_contactorN_state(idx);
    }

    return rv;
}

void CbCarrierBoardRelay::set_expected_feedback_change(bool on) {
    std::scoped_lock lock(this->expected_edge_mutex);

    this->set_expected_edge(on);
}

std::chrono::milliseconds CbCarrierBoardRelay::get_closing_delay_left() const {
    return 0ms;
}

std::ostream& CbCarrierBoardRelay::dump(std::ostream& os) const {
    os << this->relay_name << " (" << (this->get_actuator_state() ? "CLOSED" : "OPEN") << ", ";
    if (this->has_feedback) {
        os << (this->get_feedback_state() ? "CLOSED" : "OPEN");
    } else {
        os << "UNUSED";
    }
    os << ")";
    return os;
}
