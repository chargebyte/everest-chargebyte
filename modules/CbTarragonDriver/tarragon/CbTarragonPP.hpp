// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once
#include <string>
#include <generated/interfaces/evse_board_support/Implementation.hpp>
#include <IIOADC.hpp>

///
/// A class for interpreting a measured ADC value for Proximity Pilot evaluation on Tarragon platform.
/// The PP contact is wired to an ADC input on Tarragon which can be read and mapped
/// according to a lookup table to the corresponding resistor value in the plugged-in socket.
/// In case the measured value is too low, i.e. the resistor is greater than specified in the
/// standard, then an exception is raised which should be caught by upper layers.
///
class CbTarragonPP {

public:
    /// @brief Default constructor.
    CbTarragonPP(void);

    /// @brief Constructor.
    /// @param adc_device The name of the IIO ADC device to use.
    /// @param adc_device_channel The name of the channel of the ADC device, e.g. 'voltage5'.
    CbTarragonPP(const std::string& adc_device, const std::string& adc_device_channel);

    /// @brief Reads the current cable rating from underlying ADC
    ///        Throws a `std::underflow_error`exception in case the ADC reading is outside of
    ///        the expected range and cannot be mapped to a result value.
    /// @param[out] voltage The physically measured voltage at the ADC input pin in mV
    /// @return A cable current rating using enum types::board_support_common::Ampacity
    types::board_support_common::Ampacity get_ampacity(int& voltage);

private:
    /// @brief The underlying IIO ADC instance.
    IIOADC adc;
};
