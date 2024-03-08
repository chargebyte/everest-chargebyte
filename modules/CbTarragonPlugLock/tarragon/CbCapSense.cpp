#include "CbCapSense.hpp"
#include "IIOADCUtils.hpp"

#define ADC_MAX_VALUE ((1 << 12) - 1) // max value at ADC precision of 12bit
#define ADC_REF_VOLTAGE 3300 // reference voltage which is used to scale capacitor voltage
#define CAP_SCALE_VALUE (6.8 / 2.2 + 1) // reference voltage which is used to scale capacitor voltage (see schematic of Tarragon hardware)
#define CAP_MAX_VOLTAGE 12000 // max capacitor voltage

CbCapSense::CbCapSense(void) {
}

CbCapSense::CbCapSense(const std::string& adc_device, const std::string& adc_channel,
                        int charged_threshold_voltage):
    adc(get_iioadc_by_name(adc_device)) {
    
    // range check for threshold values
    this->check_range(charged_threshold_voltage, 0, CAP_MAX_VOLTAGE);
    this->charged_threshold_voltage = charged_threshold_voltage;

    // open adc channel
    if (!this->adc.has_channel(adc_channel))
        throw std::runtime_error("IIO ADC with name '" + adc_device + "' has no channel '" + adc_channel + "'.");

    this->adc.open_channel(adc_channel);
}

bool CbCapSense::is_charged(void) {
    if (this->get_voltage() <= this->charged_threshold_voltage)
        return false;

    return true;
}

int CbCapSense::get_voltage(void) {
    // read current value from hardware and convert to human-friendly one
    return this->calc_voltage(this->adc.get_value());
}

void CbCapSense::check_range(int charged_threshold_voltage, int min, int max) const {
    if ((charged_threshold_voltage < min) || (charged_threshold_voltage > max))
        throw std::out_of_range("charged_threshold_voltage(" + std::to_string(charged_threshold_voltage) +
                                ") not in range between 0 and " + std::to_string(CAP_MAX_VOLTAGE) + " mV");
}

int CbCapSense::calc_voltage(int adc_value) const {
    // scale value to ADC range
    int v_int = adc_value * ADC_REF_VOLTAGE / ADC_MAX_VALUE;

    // scale ADC voltage to voltage for motor driver
    int v_cap = v_int * CAP_SCALE_VALUE;
    return v_cap;
}