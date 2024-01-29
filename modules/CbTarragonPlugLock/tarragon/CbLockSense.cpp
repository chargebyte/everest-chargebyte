#include "CbLockSense.hpp"
#include "IIOADCUtils.hpp"

#define ADC_MAX_VALUE ((1 << 12) - 1) // max value at ADC precision of 12bit
#define ADC_REF_VOLTAGE 3300 // reference voltage which is used by ADC

CbLockSense::CbLockSense(void) {
}

CbLockSense::CbLockSense(const std::string& adc_device, const std::string& adc_channel,
                         int unlock_threshold, int lock_threshold):
    adc(get_iioadc_by_name(adc_device)),
    lock_threshold(lock_threshold),
    unlock_threshold(unlock_threshold) {

        if (!this->adc.has_channel(adc_channel))
            throw std::runtime_error("IIO ADC with name '" + adc_device + "' has no channel '" + adc_channel + "'.");

        this->adc.open_channel(adc_channel);
}

int CbLockSense::get_value(void) {
    // read current value from hardware and convert to human-friendly one
    return this->calc_voltage(this->adc.get_value());
}

bool CbLockSense::is_locked(void) {
    int voltage = get_value();

    return (voltage >= this->lock_threshold && voltage <= 3300);
}

bool CbLockSense::is_unlocked(void) {
    int voltage = get_value();

    return (voltage >= 0 && voltage <= this->unlock_threshold);
}

int CbLockSense::calc_voltage(int adc_value) const {
    // scale value to ADC range
    int v_int = adc_value * ADC_REF_VOLTAGE / ADC_MAX_VALUE;
    return v_int;
}
