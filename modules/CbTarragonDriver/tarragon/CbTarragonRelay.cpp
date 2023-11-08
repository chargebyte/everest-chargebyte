#include <memory>
#include <stdexcept>
#include <gpiod.hpp>
#include <gpiodUtils.hpp>
#include "CbTarragonRelay.hpp"

using namespace std::chrono_literals;

CbTarragonRelay::CbTarragonRelay(void) {}

CbTarragonRelay::CbTarragonRelay(const std::string &relay_name,
                                 const std::string &actuator_gpio_line_name,
                                 const std::string &feedback_type,
                                 const std::string &feedback_gpio_line_name) {
    bool actuator_active_low{false};
    bool feedback_active_low{false};

    this->relay_name = relay_name;

    // find if the name has the character '/' and find it position. This is done because the
    // Kernal throws warning when a GPIO consumer has '/' in its name.
    size_t pos = this->relay_name.find('/');

    // if '/' is found, replace it with '-'
    if (pos != std::string::npos)
        this->relay_name.replace(pos, 1, "-");

    this->feedback_type = this->get_feedback_type(feedback_type);

    this->delay_contactor_close = false;
    this->last_contactor_open_ts = std::chrono::steady_clock::now();
    this->contactor_close_interval = 10s;

    // initialize the buffer to be of a capacity '1' so that we read a single event
    this->feedback_event_buffer = gpiod::edge_event_buffer(1);

    this->actuator = std::make_unique <gpiod::line_request> (get_gpioline_by_name(actuator_gpio_line_name,
                                                                                  this->relay_name,
                                                                                  gpiod::line_settings()
                                                                                       .set_direction(gpiod::line::direction::OUTPUT)
                                                                                       .set_output_value(gpiod::line::value::INACTIVE)
                                                                                       .set_active_low(actuator_active_low)));

    if (this->feedback_type != CbContactorFeedbackType::NONE) {
        this->feedback = std::make_unique <gpiod::line_request> (get_gpioline_by_name(feedback_gpio_line_name,
                                                                                      this->relay_name,
                                                                                      gpiod::line_settings()
                                                                                           .set_direction(gpiod::line::direction::INPUT)
                                                                                           .set_edge_detection(gpiod::line::edge::BOTH)
                                                                                           .set_active_low(feedback_active_low)));
    }
}

void CbTarragonRelay::set_last_contactor_open_ts(std::chrono::time_point<std::chrono::steady_clock> timestamp) {
    this->last_contactor_open_ts = timestamp;
}

void CbTarragonRelay::set_delay_contactor_close(bool value) {
    this->delay_contactor_close = value;
}

bool CbTarragonRelay::can_close_contactor(void) {
    if (this->delay_contactor_close == false)
        return true;

    if (std::chrono::steady_clock::now() - this-> last_contactor_open_ts < this->contactor_close_interval)
        return false;

    this->delay_contactor_close = false;
    return true;
}

std::chrono::seconds CbTarragonRelay::get_contactor_close_interval(void) {
    return this->contactor_close_interval;
}

void CbTarragonRelay::set_actuator_state(bool new_state_on) {
    this->actuator->set_value(this->actuator->offsets()[0], new_state_on ?
                                                                       gpiod::line::value::ACTIVE :
                                                                       gpiod::line::value::INACTIVE);
}

bool CbTarragonRelay::get_actuator_state(void) {
    return this->actuator->get_value(this->actuator->offsets()[0])
            == gpiod::line::value::ACTIVE;
}

CbContactorFeedbackType CbTarragonRelay::get_feedback_type(std::string feedback_type) {
    CbContactorFeedbackType rv_feedback_type = CbContactorFeedbackType::NONE;

    if (feedback_type == "no")
        rv_feedback_type = CbContactorFeedbackType::NORMALLY_OPEN;
    else if (feedback_type == "nc")
        rv_feedback_type = CbContactorFeedbackType::NORMALLY_CLOSED;

    return rv_feedback_type;
}

bool CbTarragonRelay::get_feedback_state(void) {
    bool result;

    // in case there is no feedback connected to relay, we just read the actuator state
    if (this->feedback_type == CbContactorFeedbackType::NONE)
        return this->get_actuator_state();

    result = (this->feedback->get_value(this->feedback->offsets()[0]) == gpiod::line::value::ACTIVE);

    if (this->feedback_type == CbContactorFeedbackType::NORMALLY_CLOSED)
        result = !result;

    return result;

}

bool CbTarragonRelay::wait_for_feedback(const std::chrono::nanoseconds &timeout) const {
    return this->feedback->wait_edge_events(timeout);
}

gpiod::edge_event::event_type CbTarragonRelay::read_feedback_event(void) {
    this->feedback->read_edge_events(this->feedback_event_buffer, 1);

    return this->feedback_event_buffer.get_event(0).type();
}

bool CbTarragonRelay::verify_feedback_event(gpiod::edge_event::event_type event) {
    if ((event == gpiod::edge_event::event_type::FALLING_EDGE) &&
        (this->feedback_type == CbContactorFeedbackType::NORMALLY_CLOSED))
        return 1;

    if ((event == gpiod::edge_event::event_type::RISING_EDGE) &&
        (this->feedback_type == CbContactorFeedbackType::NORMALLY_OPEN))
        return 1;

    return 0;
}
