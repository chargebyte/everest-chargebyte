// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include "CbCapSense.hpp"
#include "IIOADCUtils.hpp"
#include "CbLockSense.hpp"

CbCapSense::CbCapSense(void) {
}

CbCapSense::CbCapSense(const std::string& adc_device, const std::string& adc_channel, int charged_threshold_voltage) :
    adc(get_iioadc_by_name(adc_device)) {

    this->charged_threshold_voltage = charged_threshold_voltage;

    // open adc channel
    if (!this->adc.has_channel(adc_channel))
        throw std::runtime_error("IIO ADC with name '" + adc_device + "' has no channel '" + adc_channel + "'.");

    this->adc.open_channel(adc_channel);
}

bool CbCapSense::is_charged(void) {
    return (this->get_voltage() >= this->charged_threshold_voltage);
}

int CbCapSense::get_voltage(void) {
    // read current value from hardware and convert to human-friendly one
    return this->calc_voltage(this->adc.get_value());
}

int CbCapSense::calc_voltage(int adc_value) const {
    // scale value to ADC range
    int voltage3_3v = adc_value * ADC_REF_VOLTAGE / ADC_MAX_VALUE;

    // according to the datasheet the expected DC supply should be between 12 and 13 V
    // so scale ADC voltage according to the motor driver (max 13.5 V)
    return voltage3_3v * 90 / 22;
}

int CbCapSense::get_threshold_voltage(void) const {
    return this->charged_threshold_voltage;
}
