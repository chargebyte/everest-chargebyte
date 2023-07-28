// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#ifndef LOW_LEVEL_BOARD_SUPPORT_LOW_LEVEL_IMPL_HPP
#define LOW_LEVEL_BOARD_SUPPORT_LOW_LEVEL_IMPL_HPP

//
// AUTO GENERATED - MARKED REGIONS WILL BE KEPT
// template version 3
//

#include <generated/interfaces/board_support_low_level/Implementation.hpp>

#include "../CbTarragonDriver.hpp"

// ev@75ac1216-19eb-4182-a85c-820f1fc2c091:v1
// insert your custom include headers here
// ev@75ac1216-19eb-4182-a85c-820f1fc2c091:v1

namespace module {
namespace low_level {

struct Conf {};

class board_support_low_levelImpl : public board_support_low_levelImplBase {
public:
    board_support_low_levelImpl() = delete;
    board_support_low_levelImpl(Everest::ModuleAdapter* ev, const Everest::PtrContainer<CbTarragonDriver>& mod,
                                Conf& config) :
        board_support_low_levelImplBase(ev, "low_level"), mod(mod), config(config) {};

    // ev@8ea32d28-373f-4c90-ae5e-b4fcc74e2a61:v1
    // insert your public definitions here
    // ev@8ea32d28-373f-4c90-ae5e-b4fcc74e2a61:v1

protected:
    // command handler functions (virtual)
    virtual double handle_get_grid_current_limit() override;
    virtual int handle_get_max_phase_count() override;
    virtual void handle_enable_CP(bool& value) override;
    virtual void handle_set_duty_cycle(double& value) override;
    virtual void handle_close_contactor(bool& value) override;

    // ev@d2d1847a-7b88-41dd-ad07-92785f06f5c4:v1
    // insert your protected definitions here
    // ev@d2d1847a-7b88-41dd-ad07-92785f06f5c4:v1

private:
    const Everest::PtrContainer<CbTarragonDriver>& mod;
    const Conf& config;

    virtual void init() override;
    virtual void ready() override;

    // ev@3370e4dd-95f4-47a9-aaec-ea76f34a66c9:v1
    // insert your private definitions here
    // ev@3370e4dd-95f4-47a9-aaec-ea76f34a66c9:v1
};

// ev@3d7da0ad-02c2-493d-9920-0bbbd56b9876:v1
// insert other definitions here
// ev@3d7da0ad-02c2-493d-9920-0bbbd56b9876:v1

} // namespace low_level
} // namespace module

#endif // LOW_LEVEL_BOARD_SUPPORT_LOW_LEVEL_IMPL_HPP
