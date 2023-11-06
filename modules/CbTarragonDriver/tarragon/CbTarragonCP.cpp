#include <chrono>
#include <string>
#include <thread>
#include <generated/interfaces/evse_board_support/Implementation.hpp>
#include "CbTarragonCPADC.hpp"
#include "CbTarragonCP.hpp"

using namespace std::chrono_literals;

CbTarragonCP::CbTarragonCP(void) : peak_detector_reset_time(1500us), valid_signal_delay(2ms) {
}

CbTarragonCP::CbTarragonCP(const std::string& adc_device_pos_level, const std::string& adc_device_channel_pos_level,
                           const std::string& peak_detector_reset_gpioline_name_pos_level,
                           const std::string& adc_device_neg_level, const std::string& adc_device_channel_neg_level,
                           const std::string& peak_detector_reset_gpioline_name_neg_level) :
                           peak_detector_reset_time(1500us),
                           valid_signal_delay(2ms),
                           pos_adc(adc_device_pos_level, adc_device_channel_pos_level, peak_detector_reset_gpioline_name_pos_level),
                           neg_adc(adc_device_neg_level, adc_device_channel_neg_level, peak_detector_reset_gpioline_name_neg_level) {
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

types::board_support_common::Event CbTarragonCP::voltage_to_state(int voltage, types::board_support_common::Event previous_state) const {
    // The following thresholds mostly based on IEC 61851-1
    // Table A.4 System states detected by the charging station.
    if (voltage > 13000 /* mV */) /* > 13 V */
        return types::board_support_common::Event::MREC_14_PilotFault;

    if (voltage >= 11000 /* mV */) /* 11 V <= x <= 13 V */
        return types::board_support_common::Event::A;

    if (voltage >= 10000 /* mV */) { /* 10 V <= x < 11 V */
        switch (previous_state) {
        case types::board_support_common::Event::A:
            return types::board_support_common::Event::A;
        default:
            return types::board_support_common::Event::B;
        }
    }

    if (voltage >= 8000 /* mV */) /* 8 V <= x < 10 V */
        return types::board_support_common::Event::B;

    if (voltage >= 7000 /* mV */) { /* 7 V <= x < 8 V */
        switch (previous_state) {
        case types::board_support_common::Event::A:
        case types::board_support_common::Event::B:
            return types::board_support_common::Event::B;
        default:
            return types::board_support_common::Event::C;
        }
    }

    if (voltage >= 5000 /* mV */) /* 5 V <= x < 7 V */
        return types::board_support_common::Event::C;

    if (voltage >= 4000 /* mV */) { /* 4 V <= x < 5 V */
        switch (previous_state) {
        case types::board_support_common::Event::A:
        case types::board_support_common::Event::B:
        case types::board_support_common::Event::C:
            return types::board_support_common::Event::C;
        default:
            return types::board_support_common::Event::D;
        }
    }

    if (voltage >= 2000 /* mV */) /* 2 V <= x < 4 V */
        return types::board_support_common::Event::D;

    if (voltage >= 1000 /* mV */) { /* 1 V <= x < 2 V */
        switch (previous_state) {
        case types::board_support_common::Event::A:
        case types::board_support_common::Event::B:
        case types::board_support_common::Event::C:
        case types::board_support_common::Event::D:
            return types::board_support_common::Event::D;
        default:
            return types::board_support_common::Event::E;
        }
    }

    if (voltage >= -1000 /* mV */) /* -1 V <= x < 1 V */
        return types::board_support_common::Event::E;

    if (voltage >= -2000 /* mV */) { /* -2 V <= x < -1 V (not standard) */
        switch (previous_state) {
        case types::board_support_common::Event::A:
        case types::board_support_common::Event::B:
        case types::board_support_common::Event::C:
        case types::board_support_common::Event::D:
        case types::board_support_common::Event::E:
            return types::board_support_common::Event::E;
        default:
            return types::board_support_common::Event::MREC_14_PilotFault;
        }
    }

    if (voltage >= -10000 /* mV */) /* -10 V <= x < -2 V */
        return types::board_support_common::Event::MREC_14_PilotFault;

    if (voltage >= -11000 /* mV */) { /* -11 V <= x < -10 V (not standard) */
        return previous_state == types::board_support_common::Event::F
                   ? types::board_support_common::Event::F
                   : types::board_support_common::Event::MREC_14_PilotFault;
    }

    if (voltage >= -13000 /* mV */) /* -13 V <= x < -11 V */
        return types::board_support_common::Event::F;

    return types::board_support_common::Event::MREC_14_PilotFault; /* < -13 V */
}

bool CbTarragonCP::is_valid_cp_state(types::board_support_common::Event& cp_state) {
    return (types::board_support_common::Event::A <= cp_state) &&
               (cp_state <= types::board_support_common::Event::F);
}
