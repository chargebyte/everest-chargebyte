// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#ifndef TEMPERATURES_CB_CHARGESOM_TEMPERATURES_IMPL_HPP
#define TEMPERATURES_CB_CHARGESOM_TEMPERATURES_IMPL_HPP

//
// AUTO GENERATED - MARKED REGIONS WILL BE KEPT
// template version 3
//

#include <generated/interfaces/cb_chargesom_temperatures/Implementation.hpp>

#include "../CbParsleyDriver.hpp"

// ev@75ac1216-19eb-4182-a85c-820f1fc2c091:v1
// insert your custom include headers here
// ev@75ac1216-19eb-4182-a85c-820f1fc2c091:v1

namespace module {
namespace temperatures {

struct Conf {};

class cb_chargesom_temperaturesImpl : public cb_chargesom_temperaturesImplBase {
public:
    cb_chargesom_temperaturesImpl() = delete;
    cb_chargesom_temperaturesImpl(Everest::ModuleAdapter* ev, const Everest::PtrContainer<CbParsleyDriver>& mod,
                                  Conf& config) :
        cb_chargesom_temperaturesImplBase(ev, "temperatures"), mod(mod), config(config) {};

    // ev@8ea32d28-373f-4c90-ae5e-b4fcc74e2a61:v1
    // insert your public definitions here

    /// @brief Dtor
    ~cb_chargesom_temperaturesImpl() {
        if (this->publish_thread.joinable())
            this->publish_thread.join();
    }
    // ev@8ea32d28-373f-4c90-ae5e-b4fcc74e2a61:v1

protected:
    // no commands defined for this interface

    // ev@d2d1847a-7b88-41dd-ad07-92785f06f5c4:v1
    // insert your protected definitions here
    // ev@d2d1847a-7b88-41dd-ad07-92785f06f5c4:v1

private:
    const Everest::PtrContainer<CbParsleyDriver>& mod;
    const Conf& config;

    virtual void init() override;
    virtual void ready() override;

    // ev@3370e4dd-95f4-47a9-aaec-ea76f34a66c9:v1
    // insert your private definitions here

    /// @brief Remember whether we already reported the failed selftest
    bool selftest_failed_reported[CB_PROTO_MAX_PT1000S] = {};

    /// @brief Remember whether we already reported that this channel caused charging abort
    bool charging_abort_cause_reported[CB_PROTO_MAX_PT1000S] = {};

    /// @brief Thread for periodic publishing
    std::thread publish_thread;
    // ev@3370e4dd-95f4-47a9-aaec-ea76f34a66c9:v1
};

// ev@3d7da0ad-02c2-493d-9920-0bbbd56b9876:v1
// insert other definitions here
// ev@3d7da0ad-02c2-493d-9920-0bbbd56b9876:v1

} // namespace temperatures
} // namespace module

#endif // TEMPERATURES_CB_CHARGESOM_TEMPERATURES_IMPL_HPP
