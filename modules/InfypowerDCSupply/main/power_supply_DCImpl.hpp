// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#ifndef MAIN_POWER_SUPPLY_DC_IMPL_HPP
#define MAIN_POWER_SUPPLY_DC_IMPL_HPP

//
// AUTO GENERATED - MARKED REGIONS WILL BE KEPT
// template version 3
//

#include <generated/interfaces/power_supply_DC/Implementation.hpp>

#include "../InfypowerDCSupply.hpp"

// ev@75ac1216-19eb-4182-a85c-820f1fc2c091:v1
// insert your custom include headers here
#include <atomic>
#include <mutex>
#include <string>
// ev@75ac1216-19eb-4182-a85c-820f1fc2c091:v1

namespace module {
namespace main {

struct Conf {};

class power_supply_DCImpl : public power_supply_DCImplBase {
public:
    power_supply_DCImpl() = delete;
    power_supply_DCImpl(Everest::ModuleAdapter* ev, const Everest::PtrContainer<InfypowerDCSupply>& mod, Conf& config) :
        power_supply_DCImplBase(ev, "main"), mod(mod), config(config) {};

    // ev@8ea32d28-373f-4c90-ae5e-b4fcc74e2a61:v1
    // insert your public definitions here

    // ensure that the public method is not hidden (see below)
    using power_supply_DCImplBase::raise_error;

    /// @brief Helper to report errors.
    ///        Note: participates in overload resolution since a function with same name already exists
    ///              in the base class.
    void raise_error(const std::string& type, const std::string& sub_type, const std::string& errmsg);

    // ev@8ea32d28-373f-4c90-ae5e-b4fcc74e2a61:v1

protected:
    // command handler functions (virtual)
    virtual void handle_setMode(types::power_supply_DC::Mode& mode,
                                types::power_supply_DC::ChargingPhase& phase) override;
    virtual void handle_setExportVoltageCurrent(double& voltage, double& current) override;
    virtual void handle_setImportVoltageCurrent(double& voltage, double& current) override;

    // ev@d2d1847a-7b88-41dd-ad07-92785f06f5c4:v1
    // insert your protected definitions here
    // ev@d2d1847a-7b88-41dd-ad07-92785f06f5c4:v1

private:
    const Everest::PtrContainer<InfypowerDCSupply>& mod;
    const Conf& config;

    virtual void init() override;
    virtual void ready() override;

    // ev@3370e4dd-95f4-47a9-aaec-ea76f34a66c9:v1
    // insert your private definitions here

    /// @brief A mutex to serialize all interface calls.
    std::mutex mutex;

    /// @brief Flag to remember whether we already published a communication error.
    std::atomic_bool comm_err_reported {false};

    /// @brief Helper to report communication errors.
    void raise_comm_error(const std::string& errmsg);

    /// @brief Helper to clear reported communication errors.
    void clear_comm_error();

    /// @brief Remember last published voltage/current debug message to minimize debug output
    ///        We just store the printed message so that we don't need to compare the
    ///        underlying floats (which may have only a small change) and thus can focus on
    ///        what the developer sees.
    std::string last_vc_dbgmsg;

    // ev@3370e4dd-95f4-47a9-aaec-ea76f34a66c9:v1
};

// ev@3d7da0ad-02c2-493d-9920-0bbbd56b9876:v1
// insert other definitions here
// ev@3d7da0ad-02c2-493d-9920-0bbbd56b9876:v1

} // namespace main
} // namespace module

#endif // MAIN_POWER_SUPPLY_DC_IMPL_HPP
