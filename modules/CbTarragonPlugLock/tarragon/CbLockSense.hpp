#include <string>
#include "IIOADC.hpp"
//
// A class for abstracting the plug lock sense handling
//
class CbLockSense{

public:
    /// @brief Default constructor.
    CbLockSense(void);

    /// @brief Constructor.
    /// @param unlock_threshold The GPIO line name which controls the motor driver pin 'in1'
    CbLockSense(const std::string& adc_device,
                const std::string& adc_channel,
                int unlock_threshold_min,
                int unlock_threshold_max,
                int lock_threshold_min,
                int lock_threshold_max);

    /// @brief  Read the current value from the ADC and convert it to an integer value representing a value in mV.
    /// @return The current value in mV.
    int get_voltage(void);
    bool is_locked(void);
    bool is_unlocked(void);

private:
    /// @brief The underlying IIO ADC instance.
    IIOADC adc;

    /// @brief The threshold values for locked and unlocked state.
    int unlock_threshold_min;
    int unlock_threshold_max;
    int lock_threshold_min;
    int lock_threshold_max;

    /// @brief helper function to determine wether the value_min and max are in range
    /// @param parameter String to associate range check context for logging
    /// @param value_min The lower value to be checked
    /// @param value_max The upper value to be checked
    /// @param min The lower range limit
    /// @param max The upper range limit
    void check_range(const std::string& parameter, int value_min, int value_max, int min, int max) const;

protected:
    /// @brief  Convert a raw ADC value into a voltage
    /// @param  adc_value The raw ADC value as obtained from the IIO ADC.
    /// @return A voltage value in mV.
    int calc_voltage(int adc_value) const;
};
