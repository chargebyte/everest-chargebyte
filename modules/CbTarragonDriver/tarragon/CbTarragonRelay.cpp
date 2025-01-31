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
#include <gpiod.hpp>
#include <sigslot/signal.hpp>
#include <generated/types/cb_board_support.hpp>
#include <gpiodUtils.hpp>
#include "CbTarragonRelay.hpp"

using namespace std::chrono_literals;

CbTarragonRelay::CbTarragonRelay(const std::string& relay_name, const std::string& actuator_gpio_line_name,
                                 const std::string& feedback_gpio_line_name,
                                 const unsigned int feedback_gpio_debounce_us) :
    relay_name(relay_name),
    feedback_gpio_line_name(feedback_gpio_line_name),
    feedback_gpio_debounce_us(feedback_gpio_debounce_us) {
    // check if the relay name contains the character '/' and find its position. This is done because the
    // kernel throws a warning when a GPIO consumer has '/' in its name.
    size_t pos = this->relay_name.find('/');

    // if '/' is found, replace it with '-'
    if (pos != std::string::npos)
        this->relay_name.replace(pos, 1, "-");

    // during startup, we don't know when the relay was closed the last time;
    // in a worst-case scenario, we have a endless loop due to crash, so we
    // ensure here that the switching delay is respected
    this->last_closed_ts = std::chrono::steady_clock::now();

    // instantiate and configure the actuator gpio
    this->actuator = std::make_unique<gpiod::line_request>(
        get_gpioline_by_name(actuator_gpio_line_name, "CbTarragonRelay (" + this->relay_name + ", actuator)",
                             gpiod::line_settings()
                                 .set_direction(gpiod::line::direction::OUTPUT)
                                 .set_output_value(gpiod::line::value::INACTIVE)
                                 .set_active_low(false)));
}

CbTarragonRelay::~CbTarragonRelay() {
    // tell feedback monitoring thread to exit and not to signal unexpected state changes anymore
    this->termination_requested = true;

    // safety: explicitly open the relay, but don't wait for any feedback
    this->set_actuator_state(false, false);
}

void CbTarragonRelay::start(bool use_feedback, bool active_low) {
    if (use_feedback) {
        // instantiate and configure the feedback GPIO
        gpiod::line_settings line_settings;
        line_settings.set_direction(gpiod::line::direction::INPUT);
        line_settings.set_edge_detection(gpiod::line::edge::BOTH);
        line_settings.set_active_low(active_low);

        // passing a debounce interval of zero allows to by-pass the whole configuration
        if (this->feedback_gpio_debounce_us) {
            std::chrono::microseconds period(this->feedback_gpio_debounce_us);
            line_settings.set_debounce_period(period);
        }

        this->feedback = std::make_unique<gpiod::line_request>(get_gpioline_by_name(
            this->feedback_gpio_line_name, "CbTarragonRelay (" + this->relay_name + ", sense)", line_settings));

        // start feedback handling thread
        this->feedback_monitor = std::thread(&CbTarragonRelay::feedback_monitor_worker, this);
    }
}

void CbTarragonRelay::feedback_monitor_worker() {
    // we want to process a single event at each iteration
    gpiod::edge_event_buffer event_buffer(1);

    EVLOG_debug << "feedback_monitor_worker for '" << this->relay_name << "' started";

    while (!this->termination_requested) {
        // wait for next edge event
        if (this->feedback->wait_edge_events(this->feedback_timeout)) {
            std::scoped_lock lock(this->expected_edge_mutex);

            this->feedback->read_edge_events(event_buffer, 1);
            gpiod::edge_event::event_type ev_type = event_buffer.get_event(0).type();

            // check whether the event was expected...
            if (this->expected_edge.has_value() and !this->expected_edge_matched.has_value()) {
                // ...and whether the actual edge matches our expectation
                this->expected_edge_matched = ev_type == this->expected_edge.value();

                // signal the result to `set_actuator_state`
                this->cv_expected_edge.notify_one();

                // clear the expected edge
                this->expected_edge.reset();

                // restart waiting for events
                continue;
            }

            // ...otherwise signal unexpected feedback change
            if (!this->termination_requested)
                this->on_unexpected_change(this->relay_name, ev_type == gpiod::edge_event::event_type::RISING_EDGE);
        }
    }

    EVLOG_debug << "feedback_monitor_worker for '" << this->relay_name << "' stopped";
}

bool CbTarragonRelay::set_actuator_state(bool on, bool wait_for_feedback) {
    // Remember the desired switch direction. This must be done outside holding the lock.
    // Explanation: consider a thread calling set_actuator_state(true), i.e. it wants to
    // switch the relay on. However, it must wait `min_closing_delay`. Then this thread
    // may sleep. When during this wait time an emergency event occurs which wants to prevent
    // switching on the relay, then another thread calls set_actuator_state(false) but
    // cannot enter the critical section until the sleep of the first thread is over
    // and the previous call completed. This would result in a short glitch, ie. the first
    // thread would switch the relay on, then the next thread (emergency call) would
    // switch it off immediately. To prevent this, the second thread switches the following
    // variable to false which is then additionally considered by the first thread after the sleep.
    this->execute_actuator_state_on_switch = on;

    std::unique_lock<std::mutex> lock(this->expected_edge_mutex);

    // check whether we need to sleep and do so if so - but only when switching on
    if (on)
        std::this_thread::sleep_for(this->get_closing_delay_left());

    // the mentioned double check - return before we do actually anything
    if (on and not this->execute_actuator_state_on_switch)
        return false;

    // check whether we actually need to toggle
    if (on == this->get_actuator_state())
        return true;

    // determine whether we are expecting a state change on feedback
    this->set_expected_edge(on);

    // actually toggle the relay
    this->actuator->set_value(this->actuator->offsets()[0],
                              on ? gpiod::line::value::ACTIVE : gpiod::line::value::INACTIVE);

    // remember the timestamp when we switched on
    if (on)
        this->last_closed_ts = std::chrono::steady_clock::now();

    // in case we have no feedback to verify our operation, or we don't expect a state change,
    // or we don't want to wait, then just return success
    if (!this->feedback or !this->expected_edge.has_value() or !wait_for_feedback)
        return true;

    // sleep until the edge was seen or a timeout occurred
    this->cv_expected_edge.wait_for(lock, this->feedback_timeout,
                                    [&] { return this->expected_edge_matched.has_value(); });

    // take over the result (or false on timeout)
    return this->expected_edge_matched.value_or(false);
}

void CbTarragonRelay::set_expected_edge(bool on) {
    // if we have a feedback, let's check the current state to determine, whether we can expect a state change
    if (this->feedback and this->get_feedback_state() == this->get_actuator_state()) {
        // remember what to expect next
        this->expected_edge =
            on ? gpiod::edge_event::event_type::RISING_EDGE : gpiod::edge_event::event_type::FALLING_EDGE;
    } else {
        // no state change expected
        this->expected_edge.reset();
    }

    // reset any left over return value
    this->expected_edge_matched.reset();
}

bool CbTarragonRelay::get_actuator_state() const {
    return this->actuator->get_value(this->actuator->offsets()[0]) == gpiod::line::value::ACTIVE;
}

bool CbTarragonRelay::get_feedback_state() const {
    if (!this->feedback)
        return this->get_actuator_state();

    return this->feedback->get_value(this->feedback->offsets()[0]) == gpiod::line::value::ACTIVE;
}

void CbTarragonRelay::set_expected_feedback_change(bool on) {
    std::scoped_lock lock(this->expected_edge_mutex);

    this->set_expected_edge(on);
}

std::chrono::milliseconds CbTarragonRelay::get_closing_delay_left() const {
    auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        this->min_close_interval - (std::chrono::steady_clock::now() - this->last_closed_ts));

    // if there is still time to wait, return the amount
    if (remaining > 0ms)
        return remaining;

    // otherwise return zero
    return 0ms;
}

std::ostream& operator<<(std::ostream& os, const CbTarragonRelay& r) {
    os << r.relay_name << " (" << (r.get_actuator_state() ? "CLOSED" : "OPEN") << ", ";
    if (r.feedback) {
        os << (r.get_feedback_state() ? "CLOSED" : "OPEN");
    } else {
        os << "UNUSED";
    }
    os << ")";
    return os;
}
