// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once

#include <endian.h>
#include <cstdint>
#include <iosfwd>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

enum class AlarmTypeMask : unsigned char
{
    STANDARD = 0b00000111,
    DEVICE_ERROR = 0b11000111,
};

enum class AlarmTypeValue : unsigned char
{
    NO_ALARM = 0b000,
    PREWARNING = 0b001,
    DEVICE_ERROR = 0b00000010, // This is also specified as such in the device manual
    RESERVED_3 = 0b011,
    WARNING = 0b100,
    ALARM = 0b101,
    RESERVED_6 = 0b110,
    RESERVED_7 = 0b111,
};

enum class TestType : unsigned char
{
    NO_TEST = 0b00,
    INTERNAL_TEST = 0b01,
    EXTERNAL_TEST = 0b10,
};

class AlarmAndTestType {
public:
    /// @brief Default ctor
    AlarmAndTestType();

    /// @brief Ctor from received data
    AlarmAndTestType(unsigned char data) {
        bool found = false;

        // check whether we can map the received data to a valid alarm type
        for (const auto& alarm : this->alarm_types) {
            if ((data & static_cast<unsigned char>(alarm.second)) == static_cast<unsigned char>(alarm.first)) {
                this->value = alarm.first;
                this->mask = alarm.second;
                found = true;
            }
        }

        if (!found) {
            char err_msg[64];
            snprintf(err_msg, sizeof(err_msg), "Unknown alarm type: 0x%02hhx", data);
            throw std::range_error(err_msg);
        }

        this->test = static_cast<TestType>((data >> TEST_TYPE_SHIFT) & ((1 << TEST_TYPE_LEN) - 1));
    }

    AlarmTypeValue value {AlarmTypeValue::NO_ALARM};
    AlarmTypeMask mask {AlarmTypeMask::STANDARD};
    TestType test {TestType::NO_TEST};

private:
    /// @brief List of possible alarms combined with the mask to detect the alarm
    std::vector<std::pair<AlarmTypeValue, AlarmTypeMask>> alarm_types = {
        {AlarmTypeValue::NO_ALARM, AlarmTypeMask::STANDARD},
        {AlarmTypeValue::PREWARNING, AlarmTypeMask::STANDARD},
        {AlarmTypeValue::DEVICE_ERROR, AlarmTypeMask::DEVICE_ERROR},
        {AlarmTypeValue::WARNING, AlarmTypeMask::STANDARD},
        {AlarmTypeValue::ALARM, AlarmTypeMask::STANDARD}};

    /// @brief The count of bits and the offset of test type within the byte
    static constexpr unsigned char TEST_TYPE_LEN = 2;
    static constexpr unsigned char TEST_TYPE_SHIFT = 6;
};

std::ostream& operator<<(std::ostream& os, const AlarmAndTestType& alarm);

enum class Unit : unsigned char
{
    INVALID_INIT = 0b00000,
    NO_UNIT = 0b00001,
    OHM = 0b00010,
    AMPERE = 0b00011,
    VOLT = 0b00100,
    PERCENT = 0b00101,
    HERTZ = 0b00110,
    BAUD = 0b00111,
    FARAD = 0b01000,
    HENRY = 0b01001,
    DEGREE_CELSIUS = 0b01010,
    DEGREE_FAHRENHEIT = 0b01011,
    SECOND = 0b01100,
    MINUTE = 0b01101,
    HOUR = 0b01110,
    DAY = 0b01111,
    MONTH = 0b10000,
};

std::ostream& operator<<(std::ostream& os, const Unit& u);

enum class RangeValidity : unsigned char
{
    ACTUAL_VALUE = 0b00,
    LOWER_VALUE = 0b01,
    HIGHER_VALUE = 0b10,
    INVALID_VALUE = 0b11,
};

std::ostream& operator<<(std::ostream& os, const RangeValidity& r);

class RangeAndUnit {
public:
    /// @brief Default ctor
    RangeAndUnit();

    /// @brief Ctor from received data
    RangeAndUnit(unsigned char data) {
        this->unit = static_cast<Unit>((data >> UNIT_SHIFT) & ((1 << UNIT_LEN) - 1));
        this->validity = static_cast<RangeValidity>((data >> RANGE_VALIDITY_SHIFT) & ((1 << RANGE_VALIDITY_LEN) - 1));
    }

    Unit unit {Unit::INVALID_INIT};
    RangeValidity validity {RangeValidity::INVALID_VALUE};

private:
    /// @brief The count of bits and the offset of unit field within the byte
    static constexpr unsigned char UNIT_LEN = 5;
    static constexpr unsigned char UNIT_SHIFT = 0;

    /// @brief The count of bits and the offset of range validity field within the byte
    static constexpr unsigned char RANGE_VALIDITY_LEN = 2;
    static constexpr unsigned char RANGE_VALIDITY_SHIFT = 6;
};

class ChannelDescription {
public:
    /// @brief Default ctor
    ChannelDescription();

    /// @brief Ctor from received data
    ChannelDescription(unsigned int data) : value(data), description(ChannelDescription::MAP.at(data)) {
    }

    unsigned char value;
    std::string description;

private:
    inline static const std::map<unsigned int, std::string> MAP = {
        {1, "Insulation fault"},   {71, "Insulation fault"}, {76, "Voltage"},          {77, "Undervoltage"},
        {78, "Overvoltage"},       {82, "Capacitance"},      {86, "Insulation fault"}, {101, "System connection"},
        {102, "Earth connection"}, {115, "Device error"},    {129, "Device error"},    {145, "Own address"},
    };
};

class Measurement {
public:
    /// @brief Default ctor
    Measurement();

    /// @brief Ctor from received data
    Measurement(std::vector<int32_t> data) {
        // the single integers (originally u16) in data are already converted to host byte order
        this->value = static_cast<float>((data.at(1) << 16) | data.at(0));

        this->alarm = AlarmAndTestType(data.at(2) & 0xff);

        this->unit = RangeAndUnit((data.at(2) >> 8) & 0xff);

        this->desc = ChannelDescription(data.at(3));
    }

    float value {0.0f};
    AlarmAndTestType alarm;
    RangeAndUnit unit;
    ChannelDescription desc;
};

std::ostream& operator<<(std::ostream& os, const Measurement& m);
