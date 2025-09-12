// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#ifndef EVSE_BOARD_SUPPORT_EVSE_BOARD_SUPPORT_IMPL_HPP
#define EVSE_BOARD_SUPPORT_EVSE_BOARD_SUPPORT_IMPL_HPP

//
// AUTO GENERATED - MARKED REGIONS WILL BE KEPT
// template version 3
//

#include <generated/interfaces/evse_board_support/Implementation.hpp>

#include "../CbParsleyDriver.hpp"

// ev@75ac1216-19eb-4182-a85c-820f1fc2c091:v1
// insert your custom include headers here

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
// B0 is defined in terminios.h for UART baudrate, but in CEState for MCS too - so undefine it before the inclusion
#undef B0
#include <generated/types/cb_board_support.hpp>
// ev@75ac1216-19eb-4182-a85c-820f1fc2c091:v1

namespace module {
namespace evse_board_support {

struct Conf {};

class evse_board_supportImpl : public evse_board_supportImplBase {
public:
    evse_board_supportImpl() = delete;
    evse_board_supportImpl(Everest::ModuleAdapter* ev, const Everest::PtrContainer<CbParsleyDriver>& mod,
                           Conf& config) :
        evse_board_supportImplBase(ev, "evse_board_support"), mod(mod), config(config) {};

    // ev@8ea32d28-373f-4c90-ae5e-b4fcc74e2a61:v1
    // insert your public definitions here
    // ev@8ea32d28-373f-4c90-ae5e-b4fcc74e2a61:v1

protected:
    // command handler functions (virtual)
    virtual void handle_enable(bool& value) override;
    virtual void handle_pwm_on(double& value) override;
    virtual void handle_pwm_off() override;
    virtual void handle_pwm_F() override;
    virtual void handle_allow_power_on(types::evse_board_support::PowerOnOff& value) override;
    virtual void handle_ac_switch_three_phases_while_charging(bool& value) override;
    virtual void handle_evse_replug(int& value) override;
    virtual types::board_support_common::ProximityPilot handle_ac_read_pp_ampacity() override;
    virtual void handle_ac_set_overcurrent_limit_A(double& value) override;

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

    /// @brief Hardware Capabilities
    types::evse_board_support::HardwareCapabilities hw_capabilities;

    /// @brief Tracks whether this EVSE is enabled or not.
    std::atomic_bool is_enabled {false};

    /// @brief Tracks the last published CP state.
    types::cb_board_support::CPState cp_current_state {types::cb_board_support::CPState::PowerOn};

    /// @brief Mutex to protect `cp_current_state`.
    std::mutex cp_mutex;

    /// @brief Flag to remember the desired and reported contactor state.
    ///        Since there is no contactor controlling on our side, we use this flag
    ///        to remember the current state and judge whether we have to report the
    ///        state change.
    std::atomic_bool contactor_state {false};

    /// @brief Remember the raised error.
    Everest::error::Error last_reported_fault;

    /// @brief Flag that we raised an error (common flag for all not-yet-covered ones).
    bool generic_fault_reported {false};
    // ev@3370e4dd-95f4-47a9-aaec-ea76f34a66c9:v1
};

// ev@3d7da0ad-02c2-493d-9920-0bbbd56b9876:v1
// insert other definitions here
// ev@3d7da0ad-02c2-493d-9920-0bbbd56b9876:v1

} // namespace evse_board_support
} // namespace module

#endif // EVSE_BOARD_SUPPORT_EVSE_BOARD_SUPPORT_IMPL_HPP
