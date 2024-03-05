// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#ifndef CB_TARRAGON_PLUG_LOCK_HPP
#define CB_TARRAGON_PLUG_LOCK_HPP

//
// AUTO GENERATED - MARKED REGIONS WILL BE KEPT
// template version 2
//

#include "ld-ev.hpp"

// headers for provided interface implementations
#include <generated/interfaces/connector_lock/Implementation.hpp>

// ev@4bf81b14-a215-475c-a1d3-0a484ae48918:v1
// insert your custom include headers here
// ev@4bf81b14-a215-475c-a1d3-0a484ae48918:v1

namespace module {

struct Conf {
    std::string sense_adc_device;
    std::string sense_adc_channel;
    int actuator_duration;
    int unlocked_threshold_voltage_min;
    int unlocked_threshold_voltage_max;
    int locked_threshold_voltage_min;
    int locked_threshold_voltage_max;
    std::string drv8872_in1_gpio_line_name;
    std::string drv8872_in2_gpio_line_name;
    bool drv8872_in1_active_low;
    bool drv8872_in2_active_low;
};

class CbTarragonPlugLock : public Everest::ModuleBase {
public:
    CbTarragonPlugLock() = delete;
    CbTarragonPlugLock(const ModuleInfo& info, std::unique_ptr<connector_lockImplBase> p_connector_lock, Conf& config) :
        ModuleBase(info), p_connector_lock(std::move(p_connector_lock)), config(config) {};

    const std::unique_ptr<connector_lockImplBase> p_connector_lock;
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

#endif // CB_TARRAGON_PLUG_LOCK_HPP