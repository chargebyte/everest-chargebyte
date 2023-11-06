#pragma once
#include <chrono>
#include <string>
#include <generated/interfaces/evse_board_support/Implementation.hpp>
#include "CbTarragonCPADC.hpp"

///
/// A class for abstracting the Control Pilot observation on Tarragon platform.
/// On Tarargon platform, the PWM observation circuit uses two peak detectors
/// to measure the positive and negative side of the (PWM) signal.
/// This class encapsulates the platform specific details, e.g. timings to
/// respect when accessing the ADC values.
/// The reading returned by this class via `get_values` are already converted
/// to a real world voltage in mV.
///
class CbTarragonCP {

public:
    /// @brief Default constructor.
    CbTarragonCP(void);

    /// @brief Constructor.
    /// @param adc_device_pos_level The name of the IIO ADC device to use for the positive signal side.
    /// @param adc_device_channel_pos_level The name of the channel of the ADC device, e.g. 'voltage5' for the positive signal side.
    /// @param peak_detector_reset_gpioline_name_pos_level The name of the GPIO line to reset the positive peak detector circuit.
    /// @param adc_device_neg_level The name of the IIO ADC device to use for the negative signal side.
    /// @param adc_device_channel_neg_level The name of the channel of the ADC device, e.g. 'voltage1' for the negative signal side.
    /// @param peak_detector_reset_gpioline_name_neg_level The name of the GPIO line to reset the negative peak detector circuit.
    CbTarragonCP(const std::string& adc_device_pos_level,
                 const std::string& adc_device_channel_pos_level,
                 const std::string& peak_detector_reset_gpioline_name_pos_level,
                 const std::string& adc_device_neg_level,
                 const std::string& adc_device_channel_neg_level,
                 const std::string& peak_detector_reset_gpioline_name_neg_level);

    /// @brief  Read the current Control Pilot voltage values, i.e. both signal sides.
    /// @param positive_value Reference which will be updated with the current value of the positive side (in mV).
    /// @param negative_value Reference which will be updated with the current value of the negative side (in mV).
    void get_values(int& positive_value, int& negative_value);

    /// @brief Helper to map a measured voltage to a CP state (takes hysteresis into account)
    types::board_support_common::Event voltage_to_state(int voltage, types::board_support_common::Event previous_state) const;

    /// @brief Helper to judge whether CP state is invalid or in range A..F
    bool is_valid_cp_state(types::board_support_common::Event& cp_state);

private:
    /// @brief The ADC for positive side of the CP signal.
    CbTarragonCPADC pos_adc;

    /// @brief The ADC for negative side of the CP signal.
    CbTarragonCPADC neg_adc;

    /// @brief The time in microseconds which is required for the peak detector reset to complete.
    ///        The hardware requires at least 1.5 ms (inclusive safety margin).
     std::chrono::microseconds peak_detector_reset_time;

    /// @brief The time in microseconds which is required for a stabilized signal after peak detector was reset.
    ///        The signal would be invalid for 2 ms (inclusive safety margin).
     std::chrono::microseconds valid_signal_delay;

};
