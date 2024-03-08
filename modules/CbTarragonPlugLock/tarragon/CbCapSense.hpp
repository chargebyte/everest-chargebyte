#include <string>
#include "IIOADC.hpp"
//
// A class for abstracting the capacitor sense handling
//
class CbCapSense{

public:
    /// @brief Default constructor.
    CbCapSense(void);

    /// @brief Constructor.
    /// @param adc_device The name of the IIO ADC device to use.
    /// @param adc_channel The name of the channel of the ADC device, e.g. 'voltage4'.
    /// @param charged_threshold_voltage The threshold where capacitors are indicated as charged.
    CbCapSense(const std::string& adc_device,
                const std::string& adc_channel,
                int charged_threshold_voltage);

    /// @brief Check wether the capacitors are charged
    /// @return Returns true if capacitors voltage is above charged_threshold_voltage
    bool is_charged(void);

    /// @brief Read the current value from the ADC and convert it to an integer value representing a value in mV.
    /// @return The current value in mV.
    int get_voltage(void);

private:
    /// @brief max capacitor voltage
    static constexpr unsigned int CAP_MAX_VOLTAGE {12000};

    /// @brief reference voltage which is used to scale capacitor voltage
    static constexpr unsigned int ADC_REF_VOLTAGE {3300};

    /// @brief max value at ADC precision of 12bit
	static constexpr unsigned int ADC_MAX_VALUE {(1 << 12) - 1};

    /// @brief The underlying IIO ADC instance.
    IIOADC adc;

    /// @brief The threshold voltage in mV to determine charged capacitors
    int charged_threshold_voltage;

    /// @brief setter function for charged_threshold_voltage which also does a range check
    /// @param charged_threshold_voltage The threshold voltage in mV
    void set_threshold_voltage(int charged_threshold_voltage);

protected:
    /// @brief  Convert a raw ADC value into a voltage scaled to 12V
    /// @param  adc_value The raw ADC value as obtained from the IIO ADC.
    /// @return A voltage value in mV.
    int calc_voltage(int adc_value) const;
};
