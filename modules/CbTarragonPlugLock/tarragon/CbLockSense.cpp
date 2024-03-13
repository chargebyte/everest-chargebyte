#include "CbLockSense.hpp"
#include "IIOADCUtils.hpp"
#include <everest/logging.hpp>

CbLockSense::CbLockSense(void) {
}

CbLockSense::CbLockSense(const std::string& adc_device, const std::string& adc_channel,
                         int unlock_threshold_min, int unlock_threshold_max,
                         int lock_threshold_min, int lock_threshold_max):
    adc(get_iioadc_by_name(adc_device)) {

    // range check for threshold values
    this->check_range("unlock_threshold", unlock_threshold_min, unlock_threshold_max, 0, ADC_REF_VOLTAGE);
    this->unlock_threshold_min = unlock_threshold_min;
    this->unlock_threshold_max = unlock_threshold_max;

    this->check_range("lock_threshold", lock_threshold_min, lock_threshold_max, 0, ADC_REF_VOLTAGE);
    this->lock_threshold_min = lock_threshold_min;
    this->lock_threshold_max = lock_threshold_max;

    // open adc channel
    if (!this->adc.has_channel(adc_channel))
        throw std::runtime_error("IIO ADC with name '" + adc_device + "' has no channel '" + adc_channel + "'.");

    this->adc.open_channel(adc_channel);
}

int CbLockSense::get_voltage(void) {
    // read current value from hardware and convert to human-friendly one
    return this->calc_voltage(this->adc.get_value());
}

bool CbLockSense::is_locked(void) {
    int voltage = this->get_voltage();

    return (voltage >= this->lock_threshold_min && voltage <= this->lock_threshold_max);
}

bool CbLockSense::is_unlocked(void) {
    int voltage = this->get_voltage();

    return (voltage >= this->unlock_threshold_min && voltage <= this->unlock_threshold_max);
}

int CbLockSense::calc_voltage(int adc_value) const {
    // scale value to ADC range
    int v_int = adc_value * ADC_REF_VOLTAGE / ADC_MAX_VALUE;
    return v_int;
}

void CbLockSense::check_range(const std::string& parameter, int value_min, int value_max, int min, int max) const {

    if ((value_min < min) || (value_min > max))
        throw std::out_of_range(parameter + "_min(" + std::to_string(value_min) +
                                ") not in range between 0 and " + std::to_string(ADC_REF_VOLTAGE) + " mV");

    if ((value_max < min) || (value_max > max))
        throw std::out_of_range(parameter + "_max(" + std::to_string(value_max) +
                                ") not in range between 0 and " + std::to_string(ADC_REF_VOLTAGE) + " mV");

    if ((value_min > value_max))
        throw std::out_of_range(parameter + "_min(" + std::to_string(value_min) +
                                " mV) > " + parameter + "_max(" + std::to_string(value_max) + " mV)");
}
