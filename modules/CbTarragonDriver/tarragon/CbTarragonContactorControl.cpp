// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include <chrono>
#include <optional>
#include <string>
#include <gpiod.hpp>
#include <everest/logging.hpp>
#include <iostream>
#include "CbTarragonContactorControl.hpp"

CbTarragonContactorControl::CbTarragonContactorControl(void) {
}

CbTarragonContactorControl::CbTarragonContactorControl(
    const std::string& relay_1_name, const std::string& relay_1_actuator_gpio_line_name,
    const std::string& relay_1_feedback_gpio_line_name, const unsigned int relay_1_gpio_debounce_us,
    const std::string& contactor_1_feedback_type, const std::string& relay_2_name,
    const std::string& relay_2_actuator_gpio_line_name, const std::string& relay_2_feedback_gpio_line_name,
    const unsigned int relay_2_gpio_debounce_us, const std::string& contactor_2_feedback_type,
    bool switch_3ph1ph_enabled) :
    relay_1(relay_1_name, relay_1_actuator_gpio_line_name, contactor_1_feedback_type, relay_1_feedback_gpio_line_name,
            relay_1_gpio_debounce_us),
    contactor_1_feedback_type(contactor_1_feedback_type) {

    EVLOG_info << "Primary contactor feedback type: '" << this->contactor_1_feedback_type << "'";

    if (this->contactor_1_feedback_type == "none")
        EVLOG_warning << "The primary contactor has the feedback pin not connected. This is not recommended.";

    // only instantiate the second relay if the user actually wants to use phase-switching
    // (it might happen that the second relay would be used for other purposes which
    // should be handled by another piece of software then)
    if (switch_3ph1ph_enabled) {
        this->relay_2 = CbTarragonRelay(relay_2_name, relay_2_actuator_gpio_line_name, contactor_2_feedback_type,
                                        relay_2_feedback_gpio_line_name, relay_2_gpio_debounce_us);

        this->contactor_2_feedback_type = contactor_2_feedback_type;
        EVLOG_info << "Secondary contactor feedback type: '" << this->contactor_2_feedback_type << "'";

        if (this->contactor_2_feedback_type == "none")
            EVLOG_warning << "The secondary contactor has the feedback pin not connected. This is not recommended.";
    }

    // if phase-count switching is enabled, we default to 3-phase operation
    this->target_phase_count = switch_3ph1ph_enabled ? 3 : 1;

    // initialize the actual state by reading the GPIO feedback
    this->actual_state = this->get_current_state();
    // the state must be OPEN
    if (this->actual_state != ContactorState::CONTACTOR_OPEN)
        EVLOG_error << "Contactor is not in OPEN state during initialization";

    this->new_target_state_ts = std::chrono::steady_clock::now();
    this->is_new_target_state_set = false;

    this->last_actual_state_open_ts = std::chrono::steady_clock::now();
    this->delay_contactor_close = false;

    // making sure that we have no contactor errors by forcing a safe initial state
    this->set_target_state(ContactorState::CONTACTOR_OPEN);
}

ContactorState CbTarragonContactorControl::get_current_state() {
    if (this->target_phase_count == 1) {
        // when target_phase_count == 1 then this means that we either want to switch only
        // a single phase, or we don't have phase-count switching setup at all;
        // but in a phase-count switching setup, we want to ensure that only the primary
        // contactor is closed so in case the secondary is also closed report CONTACTOR_UNKNOWN
        if (this->relay_2.has_value() && this->relay_2.value().get_feedback_state())
            return ContactorState::CONTACTOR_UNKNOWN;

        return this->relay_1.get_feedback_state() ? ContactorState::CONTACTOR_CLOSED : ContactorState::CONTACTOR_OPEN;
    }

    // reaching this line means: phase-count switching setup
    // so not having the second relay configured is error -> report as UNKNOWN
    if (!this->relay_2.has_value())
        return ContactorState::CONTACTOR_UNKNOWN;

    return (this->relay_1.get_feedback_state() && this->relay_2.value().get_feedback_state())
               ? ContactorState::CONTACTOR_CLOSED
               : ContactorState::CONTACTOR_OPEN;
}

ContactorState CbTarragonContactorControl::get_state(StateType state) const {
    if (state == StateType::ACTUAL_STATE)
        return this->actual_state;
    else if (state == StateType::TARGET_STATE)
        return this->target_state;

    return ContactorState::CONTACTOR_UNKNOWN;
}

void CbTarragonContactorControl::set_target_state(ContactorState target_state) {
    bool actuator_target_state = (target_state == ContactorState::CONTACTOR_CLOSED ? 1 : 0);
    this->target_state = target_state;

    if ((this->actual_state == this->target_state) && (this->target_state != ContactorState::CONTACTOR_OPEN))
        return;

    // avoid very fast switching in order to ensure internal relay lifetime
    if ((this->target_state == ContactorState::CONTACTOR_CLOSED) && (this->is_switch_on_allowed() == false)) {

        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                                  this->get_last_actual_state_open_ts())
                                .count();
        auto remaining_time = this->relay_1.get_contactor_close_interval() -
                              std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::steady_clock::now() - this->get_last_actual_state_open_ts());

        EVLOG_warning << "Attempt to close contactor within " << elapsed_time
                      << " ms from last CLOSED state! Delay for " << remaining_time.count() << " ms";
        this->delay_contactor_close = true;
        return;
    }

    // capture the timestamp of the new actuator target state
    this->new_target_state_ts = std::chrono::steady_clock::now();
    this->is_new_target_state_set = true;

    // switching 2nd relay in the following is only required when phase-count switching is enabled
    // and this usually requires the relay 2 to be present - however, we can check anyway

    switch (this->target_state) {
    case ContactorState::CONTACTOR_OPEN:
        // relay 1 must be opened first
        this->relay_1.set_actuator_state(actuator_target_state);

        // then we can also relay 2
        if (this->relay_2.has_value())
            this->relay_2.value().set_actuator_state(actuator_target_state);

        break;

    case ContactorState::CONTACTOR_CLOSED:
        // closing relay 2 first is important
        if (this->target_phase_count == 3 && this->relay_2.has_value())
            this->relay_2.value().set_actuator_state(actuator_target_state);

        // then close relay 1
        this->relay_1.set_actuator_state(actuator_target_state);

        break;

    default:
        // should not happen
        ;
    }
}

void CbTarragonContactorControl::set_actual_state(ContactorState actual_state) {
    this->actual_state = actual_state;

    // update the new phase count
    this->update_actual_phase_count();

    // handle prevention of relay wear by capturing the timestamp of the last relay OPEN state
    if (this->actual_state == ContactorState::CONTACTOR_OPEN) {
        if ((this->target_phase_count == 3) && this->relay_2.has_value()) {
            this->relay_2.value().set_last_contactor_open_ts(std::chrono::steady_clock::now());
            this->relay_2.value().set_delay_contactor_close(true);
        }

        this->relay_1.set_last_contactor_open_ts(std::chrono::steady_clock::now());
        this->relay_1.set_delay_contactor_close(true);

        this->last_actual_state_open_ts = std::chrono::steady_clock::now();
    }
}

void CbTarragonContactorControl::update_actual_phase_count() {
    bool relay_1_feedback_state = this->relay_1.get_feedback_state();
    bool relay_2_feedback_state = this->relay_2.has_value() ? this->relay_2.value().get_feedback_state() : false;

    if (relay_1_feedback_state == false && relay_2_feedback_state == false)
        this->actual_phase_count = 0;

    if (relay_1_feedback_state == true && relay_2_feedback_state == false)
        this->actual_phase_count = 1;

    if (relay_1_feedback_state == true && relay_2_feedback_state == true)
        this->actual_phase_count = 3;
}

int CbTarragonContactorControl::get_actual_phase_count() const {
    return this->actual_phase_count;
}

int CbTarragonContactorControl::get_target_phase_count() const {
    return this->target_phase_count;
}

void CbTarragonContactorControl::switch_phase_count(bool use_3phases) {
    if (this->relay_2.has_value() && use_3phases)
        this->target_phase_count = 3;
    else
        this->target_phase_count = 1;
}

std::chrono::time_point<std::chrono::steady_clock> CbTarragonContactorControl::get_new_target_state_ts() const {
    return this->new_target_state_ts;
}

bool CbTarragonContactorControl::get_is_new_target_state_set() const {
    return this->is_new_target_state_set;
}

void CbTarragonContactorControl::reset_is_new_target_state_set() {
    this->is_new_target_state_set = false;
}

std::chrono::time_point<std::chrono::steady_clock> CbTarragonContactorControl::get_last_actual_state_open_ts() const {
    return this->last_actual_state_open_ts;
}

bool CbTarragonContactorControl::is_switch_on_allowed() {
    bool allowed {true};

    // avoid very fast switching in order to ensure internal relay lifetime
    if ((this->target_phase_count == 3) && this->relay_2.has_value() &&
        (this->relay_2.value().can_close_contactor() == false))
        allowed = false;

    if (this->relay_1.can_close_contactor() == false)
        allowed = false;

    return allowed;
}

bool CbTarragonContactorControl::get_delay_contactor_close() const {
    return this->delay_contactor_close;
}

void CbTarragonContactorControl::reset_delay_contactor_close() {
    this->delay_contactor_close = false;
}

bool CbTarragonContactorControl::is_error_state() const {
    if (this->actual_state != this->target_state)
        return true;

    return false;
}

bool CbTarragonContactorControl::wait_for_events(std::chrono::milliseconds duration) {
    bool event_occurred = false;

    if (this->target_phase_count == 3 && this->contactor_2_feedback_type != "none") {
        event_occurred =
            this->relay_2.value().wait_for_feedback(std::chrono::duration_cast<std::chrono::nanoseconds>(duration));

        if (this->contactor_1_feedback_type != "none") {
            event_occurred &=
                this->relay_1.wait_for_feedback(std::chrono::duration_cast<std::chrono::nanoseconds>(duration));
        }

    } else {
        if (this->contactor_1_feedback_type != "none")
            event_occurred =
                this->relay_1.wait_for_feedback(std::chrono::duration_cast<std::chrono::nanoseconds>(duration));
    }

    return event_occurred;
}

bool CbTarragonContactorControl::read_events() {
    gpiod::edge_event::event_type event;
    bool contactor_state = 0;

    if (this->target_phase_count == 3 && this->contactor_2_feedback_type != "none") {
        event = this->relay_2.value().read_feedback_event();
        contactor_state = this->relay_2.value().verify_feedback_event(event);

        if (this->contactor_1_feedback_type != "none") {
            event = this->relay_1.read_feedback_event();
            contactor_state &= this->relay_1.verify_feedback_event(event);
        }

    } else {
        if (this->contactor_1_feedback_type != "none") {
            event = this->relay_1.read_feedback_event();
            contactor_state = this->relay_1.verify_feedback_event(event);
        }
    }

    return contactor_state;
}

std::ostream& operator<<(std::ostream& os, const ContactorState& state) {
    switch (state) {
    case ContactorState::CONTACTOR_CLOSED:
        os << "CLOSED";
        break;
    case ContactorState::CONTACTOR_OPEN:
        os << "OPEN";
        break;
    default:
        os << "UNKNOWN";
        break;
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const CbTarragonContactorControl& cc) {
    ContactorState state{cc.get_state(StateType::ACTUAL_STATE)};

    os << state;
    if (state == ContactorState::CONTACTOR_CLOSED)
        os << " (phases: " << cc.get_actual_phase_count() << ")";

    return os;
}
