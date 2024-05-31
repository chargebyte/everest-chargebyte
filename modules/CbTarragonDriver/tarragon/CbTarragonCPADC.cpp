#include <memory>
#include <stdexcept>
#include <string>
#include <gpiod.hpp>
#include <gpiodUtils.hpp>
#include <IIOADC.hpp>
#include <IIOADCUtils.hpp>
#include "CbTarragonCPADC.hpp"

CbTarragonCPADC::CbTarragonCPADC(void) {
}

CbTarragonCPADC::CbTarragonCPADC(const std::string& adc_device, const std::string& adc_device_channel,
                                 const std::string& peak_detector_reset_gpio_line_name) :
    adc(get_iioadc_by_name(adc_device)) {

    if (!this->adc.has_channel(adc_device_channel))
        throw std::runtime_error("IIO ADC with name '" + adc_device + "' has no channel '" + adc_device_channel + "'.");

    this->adc.open_channel(adc_device_channel);

    this->peak_detector_reset = std::make_unique<gpiod::line_request>(
        get_gpioline_by_name(peak_detector_reset_gpio_line_name, "CbTarragonCPADC",
                             gpiod::line_settings()
                                 .set_direction(gpiod::line::direction::OUTPUT)
                                 .set_output_value(gpiod::line::value::INACTIVE)));
}

void CbTarragonCPADC::start_peak_detector_reset(void) {
    this->peak_detector_reset->set_value(this->peak_detector_reset->offsets()[0], gpiod::line::value::ACTIVE);
}

void CbTarragonCPADC::end_peak_detector_reset(void) {
    this->peak_detector_reset->set_value(this->peak_detector_reset->offsets()[0], gpiod::line::value::INACTIVE);
}

int CbTarragonCPADC::get_value(void) {
    // read current value from hardware and convert to human-friendly one
    return this->calc_voltage(this->adc.get_value());
}

int CbTarragonCPADC::calc_voltage(int adc_value) const {
    // first scale value to ADC range
    int v_int = adc_value * 3300 / 4095;

    // approximated function from circuit simulation
    // real(v) = 9,581933 * v - 16,013855
    // approximated function for integer calculation
    // real(v) = 4561/476 * v - 16013

    return v_int * 4561 / 476 - 16013;
}
