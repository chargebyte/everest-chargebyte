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
                int unlock_threshold,
                int lock_threshold);

    /// @brief  Read the current value from the ADC and convert it to an integer value representing a value in mV.
    /// @return The current value in mV.
    int get_value(void);
    bool is_locked(void);
    bool is_unlocked(void);

private:
    /// @brief The underlying IIO ADC instance.
    IIOADC adc;

    /// @brief @brief The threshold values for locked and unlocked state.
    int unlock_threshold;
    int lock_threshold;

protected:
    /// @brief  Convert a raw ADC value into a voltage
    /// @param  adc_value The raw ADC value as obtained from the IIO ADC.
    /// @return A voltage value in mV.
    int calc_voltage(int adc_value) const;
};
