// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#ifndef EVSE_HARDWARE_MANAGER_HPP
#define EVSE_HARDWARE_MANAGER_HPP

//
// AUTO GENERATED - MARKED REGIONS WILL BE KEPT
// template version 2
//

#include "ld-ev.hpp"

// headers for provided interface implementations
#include <generated/interfaces/board_support_AC/Implementation.hpp>

// headers for required interface implementations
#include <generated/interfaces/board_support_low_level/Interface.hpp>

// ev@4bf81b14-a215-475c-a1d3-0a484ae48918:v1
// insert your custom include headers here
// ev@4bf81b14-a215-475c-a1d3-0a484ae48918:v1

namespace module {

struct Conf {
    bool switch3to1phase;
    bool pluggable;
    int cable_current_limit;
};

class EvseHardwareManager : public Everest::ModuleBase {
public:
    EvseHardwareManager() = delete;
    EvseHardwareManager(const ModuleInfo& info, std::unique_ptr<board_support_ACImplBase> p_board_support,
                        std::unique_ptr<board_support_low_levelIntf> r_low_level_hardware, Conf& config) :
        ModuleBase(info),
        p_board_support(std::move(p_board_support)),
        r_low_level_hardware(std::move(r_low_level_hardware)),
        config(config) {};

    const std::unique_ptr<board_support_ACImplBase> p_board_support;
    const std::unique_ptr<board_support_low_levelIntf> r_low_level_hardware;
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

#endif // EVSE_HARDWARE_MANAGER_HPP
