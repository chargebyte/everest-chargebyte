#include <chrono>
#include <optional>
#include <string>
#include <gpiod.hpp>
#include <everest/logging.hpp>
#include <iostream>
#include "CbTarragonContactorControl.hpp"

CbTarragonContactorControl::CbTarragonContactorControl(void) {
}

CbTarragonContactorControl::CbTarragonContactorControl(const std::string &relay_1_name,
                                                       const std::string &relay_1_actuator_gpio_line_name,
                                                       const std::string &relay_1_feedback_gpio_line_name,
                                                       const std::string &contactor_1_feedback_type,
                                                       const std::string &relay_2_name,
                                                       const std::string &relay_2_actuator_gpio_line_name,
                                                       const std::string &relay_2_feedback_gpio_line_name,
                                                       const std::string &contactor_2_feedback_type) :
                                                       relay_1(relay_1_name, relay_1_actuator_gpio_line_name,
                                                               contactor_1_feedback_type, relay_1_feedback_gpio_line_name),
                                                       contactor_1_feedback_type(contactor_1_feedback_type) {

    EVLOG_info << "Primary contactor feedback type: '" << this->contactor_1_feedback_type << "'";

    if (this->contactor_1_feedback_type == "none")
        EVLOG_warning << "The primary contactor has the feedback pin not connected. This is not recommended.";


    // it might happen that the second relay would be used for other purposes other
    // than 3-phase operation. If the relay name is something else other 'R2/S2', we
    // then do not need to handle it, as this should be handled by another piece of
    // software that utilizes the relay to its need.
    if (relay_2_name == "R2/S2") {
        this->relay_2 = CbTarragonRelay(relay_2_name, relay_2_actuator_gpio_line_name,
                                        contactor_2_feedback_type, relay_2_feedback_gpio_line_name);

        this->contactor_2_feedback_type = contactor_2_feedback_type;
        EVLOG_info << "Secondary contactor feedback type: '" << this->contactor_2_feedback_type << "'";

        if (this->contactor_2_feedback_type == "none")
            EVLOG_warning << "The secondary contactor has the feedback pin not connected. This is not recommended.";
    }

    // initialize the actual state by reading the GPIO feedback
    this->actual_state = this->get_state();
    // the state must be OPEN
    if (this->actual_state != ContactorState::CONTACTOR_OPEN)
        EVLOG_error << "Contactor is not in OPEN state during initialization";

    this->new_target_state_ts = std::chrono::steady_clock::now();
    this->is_new_target_state_set = false;

    // TODO: phase switching and 3-phase operation is still yet to be implemented.
    //       For now, we force 1-phase operation
    this->actual_phase_count = 1;
    this->target_phase_count = 1;
    this->max_phase_count = 1;
    this->start_phase_switching = false;

    // making sure that we have no contactor errors by forcing a safe initial state
    this->set_target_state(ContactorState::CONTACTOR_OPEN);
}

ContactorState CbTarragonContactorControl::get_state(void) {
    if (this->target_phase_count == 1)
        return (this->relay_1.get_feedback_state() ? ContactorState::CONTACTOR_CLOSED : ContactorState::CONTACTOR_OPEN);

    if (this->relay_2.has_value())
        return (this->relay_1.get_feedback_state() && this->relay_2.value().get_feedback_state()) ?
                                                                                                  ContactorState::CONTACTOR_CLOSED :
                                                                                                  ContactorState::CONTACTOR_OPEN;
    else
        return (this->relay_1.get_feedback_state() ? ContactorState::CONTACTOR_CLOSED : ContactorState::CONTACTOR_OPEN);
}

ContactorState CbTarragonContactorControl::get_state(StateType state) {
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

    // capture the timestamp of the new actuator target state
    this->new_target_state_ts = std::chrono::steady_clock::now();
    this->is_new_target_state_set = true;

    // FIXME: phase switching and 3-phase operation is still yet to be implemented.
    //        For now, we only control one relay as we are forcing the phase count to be 1
    if ((this->target_phase_count != this->actual_phase_count) && (this->start_phase_switching == true)) {
        if (this->target_phase_count == 3) {
            if (this->relay_2.has_value())
                this->relay_2.value().set_actuator_state(actuator_target_state);
        }
        this->relay_1.set_actuator_state(actuator_target_state);

    } else {
        if (this->actual_phase_count == 3) {
            // In case a switch off is initiated
            if (this->relay_2.has_value())
                this->relay_2.value().set_actuator_state(actuator_target_state);
        }

        this->relay_1.set_actuator_state(actuator_target_state);
    }
}

void CbTarragonContactorControl::set_actual_state(ContactorState actual_state) {
    this->actual_state = actual_state;

    // update the new phase count
    this->update_actual_phase_count();
}

void CbTarragonContactorControl::update_actual_phase_count(void) {
    bool relay_1_feedback_state = this->relay_1.get_feedback_state();
    bool relay_2_feedback_state = this->relay_2.has_value() ? this->relay_2.value().get_feedback_state() : false;

    if (relay_1_feedback_state == false && relay_2_feedback_state == false)
        this->actual_phase_count = 0;

    if (relay_1_feedback_state == true && relay_2_feedback_state == false)
        this->actual_phase_count = 1;

    if (relay_1_feedback_state == true && relay_2_feedback_state == true)
        this->actual_phase_count = 3;
}

int CbTarragonContactorControl::get_actual_phase_count(void) {
    return this->actual_phase_count;
}

int CbTarragonContactorControl::get_target_phase_count(void) {
    return this->target_phase_count;
}

int CbTarragonContactorControl::get_max_phase_count(void) {
    return this->max_phase_count;
}

void CbTarragonContactorControl::set_max_phase_count(int new_max_phase_count) {
    this->max_phase_count = new_max_phase_count;
}

std::chrono::time_point<std::chrono::steady_clock> CbTarragonContactorControl::get_new_target_state_ts(void) {
    return this->new_target_state_ts;
}

bool CbTarragonContactorControl::get_is_new_target_state_set(void) {
    return this->is_new_target_state_set;
}

void CbTarragonContactorControl::reset_is_new_target_state_set(void) {
    this->is_new_target_state_set = false;
}

void CbTarragonContactorControl::start_phase_switching_while_charging(int phase_target) {
    this->target_phase_count = phase_target;
    this->start_phase_switching = true;
}

bool CbTarragonContactorControl::is_error_state(void) {
    if (this->actual_state != this->target_state)
        return true;

    return false;
}

bool CbTarragonContactorControl::wait_for_events(std::chrono::milliseconds duration) {
    bool event_occured = false;

    // FIXME: phase switching and 3-phase operation is still yet to be implemented.
    //        For now, we only control one relay as we are forcing the phase count to be 1
    if ((this->target_phase_count == 3) && (this->relay_2.has_value()) && (this->contactor_2_feedback_type != "none")) {
        event_occured = this->relay_2.value().wait_for_feedback(std::chrono::duration_cast<std::chrono::nanoseconds>(duration));

        if (this->contactor_1_feedback_type != "none")
            event_occured &= this->relay_1.wait_for_feedback(std::chrono::duration_cast<std::chrono::nanoseconds>(duration));

    } else {
        if (this->contactor_1_feedback_type != "none")
            event_occured = this->relay_1.wait_for_feedback(std::chrono::duration_cast<std::chrono::nanoseconds>(duration));
    }

    return event_occured;
}

bool CbTarragonContactorControl::read_events(void) {
    gpiod::edge_event::event_type event;
    bool contactor_state = 0;

    // FIXME: phase switching and 3-phase operation is still yet to be implemented.
    //        For now, we only control one relay as we are forcing the phase count to be 1
    if ((this->target_phase_count == 3) && (this->relay_2.has_value()) && (this->contactor_2_feedback_type != "none")) {
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
    std::string mapped_state;

    switch (state) {
    case ContactorState::CONTACTOR_CLOSED:
        mapped_state = "CLOSED";
        break;
    case ContactorState::CONTACTOR_OPEN:
        mapped_state = "OPEN";
        break;
     default:
         mapped_state = "UNKNOWN";
         break;
    }

    os << mapped_state;
    return os;
}
