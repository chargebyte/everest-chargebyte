// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest

#include <iosfwd>
#include <iostream>
#include <iomanip>
#include "BenderMeasurement.hpp"

std::ostream& operator<<(std::ostream& os, const AlarmAndTestType& alarm) {
    switch (alarm.value) {
    case AlarmTypeValue::NO_ALARM:
        os << "No Alarm";
        break;
    case AlarmTypeValue::PREWARNING:
        os << "Prewarning";
        break;
    case AlarmTypeValue::DEVICE_ERROR:
        os << "Device Error";
        break;
    case AlarmTypeValue::WARNING:
        os << "Warning";
        break;
    case AlarmTypeValue::ALARM:
        os << "Alarm";
        break;
    default:
        os << "Invalid alarm type value";
    }

    os << ", ";

    switch (alarm.test) {
    case TestType::NO_TEST:
        os << "No Test";
        break;
    case TestType::INTERNAL_TEST:
        os << "Internal Test";
        break;
    case TestType::EXTERNAL_TEST:
        os << "External Test";
        break;
    default:
        os << "Invalid test type value";
    }

    return os;
}

std::ostream& operator<<(std::ostream& os, const Unit& u) {
    switch (u) {
    case Unit::INVALID_INIT:
        os << " [Invalid (init)]";
        break;
    case Unit::NO_UNIT:
        os << "";
        break;
    case Unit::OHM:
        os << " Ω";
        break;
    case Unit::AMPERE:
        os << " A";
        break;
    case Unit::VOLT:
        os << " V";
        break;
    case Unit::PERCENT:
        os << "%";
        break;
    case Unit::HERTZ:
        os << " Hz";
        break;
    case Unit::BAUD:
        os << " baud";
        break;
    case Unit::FARAD:
        os << " F";
        break;
    case Unit::HENRY:
        os << " H";
        break;
    case Unit::DEGREE_CELSIUS:
        os << "°C";
        break;
    case Unit::DEGREE_FAHRENHEIT:
        os << "°F";
        break;
    case Unit::SECOND:
        os << " s";
        break;
    case Unit::MINUTE:
        os << " min";
        break;
    case Unit::HOUR:
        os << " h";
        break;
    case Unit::DAY:
        os << " d";
        break;
    case Unit::MONTH:
        os << " M";
        break;
    default:
        os << "Unknown";
    }

    return os;
}

std::ostream& operator<<(std::ostream& os, const RangeValidity& r) {
    switch (r) {
    case RangeValidity::ACTUAL_VALUE:
        os << "=";
        break;
    case RangeValidity::LOWER_VALUE:
        os << "≤";
        break;
    case RangeValidity::HIGHER_VALUE:
        os << "≥";
        break;
    case RangeValidity::INVALID_VALUE:
        os << "≠";
        break;
    default:
        os << "?"; // should not happen
    }

    return os;
}

std::ostream& operator<<(std::ostream& os, const Measurement& m) {
    os << m.desc.description << " (" << m.unit.validity << std::fixed << std::setprecision(1) << m.value << m.unit.unit
       << m.alarm << ")";
    return os;
}
