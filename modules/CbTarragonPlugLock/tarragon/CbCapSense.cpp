#include "CbCapSense.hpp"
#include "IIOADCUtils.hpp"

CbCapSense::CbCapSense(void) {
}

CbCapSense::CbCapSense(const std::string& adc_device, const std::string& adc_channel,
                        int charged_threshold_voltage):
    adc(get_iioadc_by_name(adc_device)) {

    this->set_threshold_voltage(charged_threshold_voltage);

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

void CbCapSense::set_threshold_voltage(int charged_threshold_voltage) {
    if ((charged_threshold_voltage < 0) || (charged_threshold_voltage > CAP_MAX_VOLTAGE))
        throw std::out_of_range("charged_threshold_voltage(" + std::to_string(charged_threshold_voltage) +
                                ") not in range between 0 and " + std::to_string(CAP_MAX_VOLTAGE) + " mV");

    this->charged_threshold_voltage = charged_threshold_voltage;
}

int CbCapSense::calc_voltage(int adc_value) const {
    // scale value to ADC range
    int v_int = adc_value * ADC_REF_VOLTAGE / ADC_MAX_VALUE;

    // scale ADC voltage to voltage for motor driver
    // TODO: Avoid floating point operation and check against Truffle
    int v_cap = v_int * 6.8 / 2.2 + 1;
    return v_cap;
}
