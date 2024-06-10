// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once
#include <string>
#include <memory>
#include <gpiod.hpp>
#include <IIOADC.hpp>
#include <IIOADCUtils.hpp>

///
/// A class for abstracting an IIO ADC device for Control Pilot on Tarragon platform.
/// On Tarargon platform, the PWM observation circuit uses a peak detector.
/// This peak detector must be reset periodically, in combination with the ADC
/// measurement.
/// This class covers these details when obtaining ADC measurements.
/// Since the Tarragon platform has two such peak detectors, one for the positive
/// and one for the negative signal side, the usage of this class is limited.
/// Reason is that both peak detector should be reset at the same time.
/// This is covered by a dedicated class on an upper layer.
/// The reading returned by this class via `get_value` is already converted
/// to a real world voltage in mV - i.e. what the signal side of the real world
/// Control Pilot really looks like.
///
class CbTarragonCPADC {

public:
    /// @brief Default constructor.
    CbTarragonCPADC(void);

    /// @brief Constructor.
    /// @param adc_device The name of the IIO ADC device to use.
    /// @param adc_device_channel The name of the channel of the ADC device, e.g. 'voltage5'.
    /// @param peak_detector_reset_gpio_line_name The name of the GPIO line to reset the peak detector circuit.
    CbTarragonCPADC(const std::string& adc_device, const std::string& adc_device_channel,
                    const std::string& peak_detector_reset_gpio_line_name);

    /// @brief Start the reset of the peak detector circuit by asserting the reset GPIO line.
    void start_peak_detector_reset(void);

    /// @brief Release the reset GPIO line.
    void end_peak_detector_reset(void);

    /// @brief  Read the current value from the ADC and convert it to an integer value representing a value in mV.
    /// @return The current value in mV.
    int get_value(void);

private:
    /// @brief The underlying IIO ADC instance.
    IIOADC adc;

    /// @brief The GPIO line to reset the peak detector circuit.
    std::unique_ptr<gpiod::line_request> peak_detector_reset;

protected:
    /// @brief  Convert a raw ADC value into a voltage
    /// @param  adc_value The raw ADC value as obtained from the IIO ADC.
    /// @return A voltage value in mV.
    int calc_voltage(int adc_value) const;
};
