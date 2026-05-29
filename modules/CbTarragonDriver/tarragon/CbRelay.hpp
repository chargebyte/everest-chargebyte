// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once
#include <chrono>
#include <ostream>
#include <string>
#include <sigslot/signal.hpp>

class CbRelay {

public:
    virtual ~CbRelay() = default;

    virtual void start(bool use_feedback, bool active_low) = 0;
    virtual bool set_actuator_state(bool on, bool wait_for_feedback) = 0;
    virtual bool get_actuator_state() const = 0;
    virtual bool get_feedback_state() const = 0;
    virtual void set_expected_feedback_change(bool on) = 0;
    virtual bool wait_for_feedback() = 0;
    virtual std::chrono::milliseconds get_closing_delay_left() const = 0;

    sigslot::signal<const std::string&, bool> on_unexpected_change;

    friend std::ostream& operator<<(std::ostream& os, const CbRelay& relay) {
        return relay.dump(os);
    }

protected:
    virtual std::ostream& dump(std::ostream& os) const = 0;
};
