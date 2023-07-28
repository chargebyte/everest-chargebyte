// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#ifndef CB_TARRAGON_DRIVER_HPP
#define CB_TARRAGON_DRIVER_HPP

//
// AUTO GENERATED - MARKED REGIONS WILL BE KEPT
// template version 2
//

#include "ld-ev.hpp"

// headers for provided interface implementations
#include <generated/interfaces/board_support_low_level/Implementation.hpp>

// ev@4bf81b14-a215-475c-a1d3-0a484ae48918:v1
// insert your custom include headers here
// ev@4bf81b14-a215-475c-a1d3-0a484ae48918:v1

namespace module {

struct Conf {
    int contactor_1_relay;
    std::string contactor_1_feedback_type;
    int contactor_2_relay;
    std::string contactor_2_feedback_type;
    std::string relay_1_name;
    int relay_1_actuator_gpio;
    bool relay_1_actuator_active_low;
    int relay_1_feedback_gpio;
    bool relay_1_feedback_active_low;
    std::string relay_2_name;
    int relay_2_actuator_gpio;
    bool relay_2_actuator_active_low;
    int relay_2_feedback_gpio;
    bool relay_2_feedback_active_low;
    int cp_rst_neg_peak;
    std::string cp_neg_peak_adc_device;
    std::string cp_neg_peak_adc_channel;
    int cp_rst_pos_peak;
    std::string cp_pos_peak_adc_device;
    std::string cp_pwm_device;
    int cp_pwmchannel;
    int cp_invert;
    std::string pp_adc_device;
    std::string pp_adc_channel;
    std::string digital_input_reference_pwm_device;
    int digital_input_reference_pwmchannel;
    bool rcd_enable;
    int rcd_gpio;
    std::string rcd_polarity;
};

class CbTarragonDriver : public Everest::ModuleBase {
public:
    CbTarragonDriver() = delete;
    CbTarragonDriver(const ModuleInfo& info, std::unique_ptr<board_support_low_levelImplBase> p_low_level,
                     Conf& config) :
        ModuleBase(info), p_low_level(std::move(p_low_level)), config(config) {};

    const std::unique_ptr<board_support_low_levelImplBase> p_low_level;
    const Conf& config;

    // ev@1fce4c5e-0ab8-41bb-90f7-14277703d2ac:v1
    // insert your public definitions here
    // ev@1fce4c5e-0ab8-41bb-90f7-14277703d2ac:v1

protected:
    // ev@4714b2ab-a24f-4b95-ab81-36439e1478de:v1
    // insert your protected definitions here
    // ev@4714b2ab-a24f-4b95-ab81-36439e1478de:v1

private:
    friend class LdEverest;
    void init();
    void ready();

    // ev@211cfdbe-f69a-4cd6-a4ec-f8aaa3d1b6c8:v1
    // insert your private definitions here
    // ev@211cfdbe-f69a-4cd6-a4ec-f8aaa3d1b6c8:v1
};

// ev@087e516b-124c-48df-94fb-109508c7cda9:v1
// insert other definitions here
// ev@087e516b-124c-48df-94fb-109508c7cda9:v1

} // namespace module

#endif // CB_TARRAGON_DRIVER_HPP
