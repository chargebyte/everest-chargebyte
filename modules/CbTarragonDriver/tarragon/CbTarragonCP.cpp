// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include <chrono>
#include <string>
#include <thread>
#include <generated/types/cb_board_support.hpp>
#include "CbTarragonCPADC.hpp"
#include "CbTarragonCP.hpp"

using namespace std::chrono_literals;

CbTarragonCP::CbTarragonCP(void) : peak_detector_reset_time(1500us), valid_signal_delay(2ms) {
}

CbTarragonCP::CbTarragonCP(const std::string& adc_device_pos_level, const std::string& adc_device_channel_pos_level,
                           const std::string& peak_detector_reset_gpioline_name_pos_level,
                           const std::string& adc_device_neg_level, const std::string& adc_device_channel_neg_level,
                           const std::string& peak_detector_reset_gpioline_name_neg_level) :
    pos_adc(adc_device_pos_level, adc_device_channel_pos_level, peak_detector_reset_gpioline_name_pos_level),
    neg_adc(adc_device_neg_level, adc_device_channel_neg_level, peak_detector_reset_gpioline_name_neg_level),
    peak_detector_reset_time(1500us),
    valid_signal_delay(2ms) {
}

void CbTarragonCP::get_values(int& positive_value, int& negative_value) {

    // reset both peak detector at the same time
    this->pos_adc.start_peak_detector_reset();
    this->neg_adc.start_peak_detector_reset();

    // hold the reset signals for the required time
    std::this_thread::sleep_for(this->peak_detector_reset_time);

    // release both reset lines
    this->pos_adc.end_peak_detector_reset();
    this->neg_adc.end_peak_detector_reset();

    // wait until both signals are valid to read
    std::this_thread::sleep_for(this->valid_signal_delay);

    positive_value = this->pos_adc.get_value();
    negative_value = this->neg_adc.get_value();
}
