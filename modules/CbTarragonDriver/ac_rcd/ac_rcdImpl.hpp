// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#ifndef AC_RCD_AC_RCD_IMPL_HPP
#define AC_RCD_AC_RCD_IMPL_HPP

//
// AUTO GENERATED - MARKED REGIONS WILL BE KEPT
// template version 3
//

#include <generated/interfaces/ac_rcd/Implementation.hpp>

#include "../CbTarragonDriver.hpp"

// ev@75ac1216-19eb-4182-a85c-820f1fc2c091:v1
#include <atomic>
#include <CbTarragonRCM.hpp>
#include <thread>
// ev@75ac1216-19eb-4182-a85c-820f1fc2c091:v1

namespace module {
namespace ac_rcd {

struct Conf {};

class ac_rcdImpl : public ac_rcdImplBase {
public:
    ac_rcdImpl() = delete;
    ac_rcdImpl(Everest::ModuleAdapter* ev, const Everest::PtrContainer<CbTarragonDriver>& mod, Conf& config) :
        ac_rcdImplBase(ev, "ac_rcd"), mod(mod), config(config) {};

    // ev@8ea32d28-373f-4c90-ae5e-b4fcc74e2a61:v1
    // insert your public definitions here
    /// @brief Deconstructor
    ~ac_rcdImpl();
    // ev@8ea32d28-373f-4c90-ae5e-b4fcc74e2a61:v1

protected:
    // command handler functions (virtual)
    virtual void handle_self_test() override;
    virtual bool handle_reset() override;

    // ev@d2d1847a-7b88-41dd-ad07-92785f06f5c4:v1
    // insert your protected definitions here
    // ev@d2d1847a-7b88-41dd-ad07-92785f06f5c4:v1

private:
    const Everest::PtrContainer<CbTarragonDriver>& mod;
    const Conf& config;

    virtual void init() override;
    virtual void ready() override;

    // ev@3370e4dd-95f4-47a9-aaec-ea76f34a66c9:v1
    /// @brief Helper to signal thread termination wish
    std::atomic_bool termination_requested {false};

    /// @brief RCM Observation thread handle
    std::thread rcm_observation_thread;

    /// @brief Member variable for holding RCM controller instance
    CbTarragonRCM rcm_controller;

    /// @brief Indicator whether RCM is still tripped
    std::atomic_bool rcm_tripped {false};

    /// @brief Main function of the RCM observation thread
    void rcm_observation_worker(void);
    // ev@3370e4dd-95f4-47a9-aaec-ea76f34a66c9:v1
};

// ev@3d7da0ad-02c2-493d-9920-0bbbd56b9876:v1
// insert other definitions here
// ev@3d7da0ad-02c2-493d-9920-0bbbd56b9876:v1

} // namespace ac_rcd
} // namespace module

#endif // AC_RCD_AC_RCD_IMPL_HPP
