#include <chrono>
#include <stdexcept>
#include <IIOADCUtils.hpp>
#include "CbTarragonPP.hpp"

CbTarragonPP::CbTarragonPP(void) {
}

CbTarragonPP::CbTarragonPP(const std::string& adc_device, const std::string& adc_device_channel) :
    adc(get_iioadc_by_name(adc_device)) {

    if (!this->adc.has_channel(adc_device_channel))
        throw std::runtime_error("IIO ADC with name '" + adc_device + "' has no channel '" + adc_device_channel + "'.");

    this->adc.open_channel(adc_device_channel);
}

types::board_support_common::Ampacity CbTarragonPP::get_ampacity(int& voltage) {
    long int adc_value; // the ADC raw value

    adc_value = this->adc.get_value();

    voltage = adc_value * 3300 / 4096;

    // map the measured value

    if (voltage >= 2459)
        return types::board_support_common::Ampacity::None; // no cable connected

    if (voltage >= 1813)
        return types::board_support_common::Ampacity::A_13;

    if (voltage >= 1170)
        return types::board_support_common::Ampacity::A_20;

    if (voltage >= 891)
        return types::board_support_common::Ampacity::A_32;

    if (voltage >= 751)
        return types::board_support_common::Ampacity::A_63_3ph_70_1ph;

    throw std::underflow_error(
        "The measured voltage for the Proximity Pilot is out-of-range (U_PP: " + std::to_string(voltage) + " mV).");
}
