#include <string>
#include "IIOADC.hpp"

/// @brief reference voltage which is used to scale capacitor voltage
static constexpr unsigned int ADC_REF_VOLTAGE {3300};

/// @brief max value at ADC precision of 12bit
static constexpr unsigned int ADC_MAX_VALUE {(1 << 12) - 1};

//
// A class for abstracting the plug lock sense handling
//
class CbLockSense{

public:
    /// @brief Default constructor.
    CbLockSense(void);

    /// @brief Constructor.
    /// @param adc_device The name of the IIO ADC device to use.
    /// @param adc_channel The name of the channel of the ADC device, e.g. 'voltage0'.
    /// @param unlock_threshold_min The lower threshold value (in mV) of a range for determinate an unlocked plug lock
    /// @param unlock_threshold_max The upper threshold value (in mV) of a range for determinate an unlocked plug lock
    /// @param lock_threshold_min The lower threshold value (in mV) of a range for determinate an locked plug lock
    /// @param lock_threshold_max The upper threshold value (in mV) of a range for determinate an locked plug lock
    CbLockSense(const std::string& adc_device,
                const std::string& adc_channel,
                int unlock_threshold_min,
                int unlock_threshold_max,
                int lock_threshold_min,
                int lock_threshold_max);

    /// @brief Read the current value from the ADC and convert it to an integer value representing a value in mV.
    /// @return The current value in mV.
    int get_voltage(void);

    /// @brief Check whether the plug lock is locked
    /// @return Returns true if plug lock is locked
    bool is_locked(void);

    /// @brief Check whether the plug lock is unlocked
    /// @return Returns true if plug lock is unlocked
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
    /// @brief  Convert a raw ADC value into a voltage scaled to 3.3 V
    /// @param  adc_value The raw ADC value as obtained from the IIO ADC.
    /// @return A voltage value in mV.
    int calc_voltage(int adc_value) const;
};
