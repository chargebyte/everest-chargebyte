// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include <chrono>
#include <iostream>
#include <string>
#include "CbContactorControl.hpp"
#include "CbContactorControlEmulation.hpp"
#include <everest/logging.hpp>

using namespace std::chrono_literals;

bool CbContactorControlEmulation::is_inconsistent_state(std::ostringstream& error_hint) const {
    return false;
}

bool CbContactorControlEmulation::switch_state(bool on) {
    this->is_on = on;
    return true;
}

bool CbContactorControlEmulation::get_state() const {
    return this->is_on;
}

std::chrono::milliseconds CbContactorControlEmulation::get_closing_delay_left() const {
    return 0ms;
}

std::ostream& CbContactorControlEmulation::dump(std::ostream& os) const {
    os << "Emulated Contactor";
    return os;
}

std::ostream& operator<<(std::ostream& os, const CbContactorControlEmulation& c) {
    return c.dump(os);
}
