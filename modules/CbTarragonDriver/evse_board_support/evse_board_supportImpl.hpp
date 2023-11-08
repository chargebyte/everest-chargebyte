// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#ifndef EVSE_BOARD_SUPPORT_EVSE_BOARD_SUPPORT_IMPL_HPP
#define EVSE_BOARD_SUPPORT_EVSE_BOARD_SUPPORT_IMPL_HPP

//
// AUTO GENERATED - MARKED REGIONS WILL BE KEPT
// template version 3
//

#include <generated/interfaces/evse_board_support/Implementation.hpp>

#include "../CbTarragonDriver.hpp"

// ev@75ac1216-19eb-4182-a85c-820f1fc2c091:v1
// insert your custom include headers here

#include <atomic>
#include <CbTarragonCP.hpp>
#include <CbTarragonPWM.hpp>
#include <CbTarragonPP.hpp>

// ev@75ac1216-19eb-4182-a85c-820f1fc2c091:v1

namespace module {
namespace evse_board_support {

struct Conf {};

class evse_board_supportImpl : public evse_board_supportImplBase {
public:
    evse_board_supportImpl() = delete;
    evse_board_supportImpl(Everest::ModuleAdapter* ev, const Everest::PtrContainer<CbTarragonDriver>& mod,
                           Conf& config) :
        evse_board_supportImplBase(ev, "evse_board_support"), mod(mod), config(config) {};

    // ev@8ea32d28-373f-4c90-ae5e-b4fcc74e2a61:v1
    // insert your public definitions here
     ~evse_board_supportImpl();
    // ev@8ea32d28-373f-4c90-ae5e-b4fcc74e2a61:v1

protected:
    // command handler functions (virtual)
    virtual void handle_setup(bool& three_phases, bool& has_ventilation, std::string& country_code) override;
    virtual types::evse_board_support::HardwareCapabilities handle_get_hw_capabilities() override;
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
    const Everest::PtrContainer<CbTarragonDriver>& mod;
    const Conf& config;

    virtual void init() override;
    virtual void ready() override;

    // ev@3370e4dd-95f4-47a9-aaec-ea76f34a66c9:v1
    // insert your private definitions here

    /// @brief Hardware Capabilities
    types::evse_board_support::HardwareCapabilities hw_capabilities;

    /// @brief Helper to signal thread termination wish
    std::atomic_bool termination_requested;

    /// @brief Control Pilot observation
    CbTarragonCP cp_controller;

    /// @brief Control Pilot generation
    CbTarragonPWM pwm_controller;

    /// @brief Mutex to enable/disable CP observation thread. Usually hold by the observation
    ///        worker thread but can be requested via `disable_cp_observation` method.
    std::mutex cp_observation_lock;

    /// @brief Helper to track whether the CP observation is running
    bool cp_observation_enabled;

    /// @brief Tracks the last published CP state.
    std::atomic<types::board_support_common::BspEvent> cp_current_state;

    /// @brief CP observation thread handle
    std::thread cp_observation_thread;

    /// @brief Proximity Pilot observation
    CbTarragonPP pp_controller;

    /// @brief Last published/detected ampacity
    types::board_support_common::ProximityPilot pp_ampacity;

    /// @brief Flag to remember whether we already published a proximity error
    std::atomic_bool pp_fault_reported;

    /// @brief Mutex to enable/disable PP observation thread
    std::mutex pp_observation_lock;

    /// @brief PP observation thread handle
    std::thread pp_observation_thread;

    /// @brief Helper to determine whether one side of the CP signal caused a CP signal change
    bool cp_state_changed(struct cp_state_signal_side& signal_side);

    /// @brief Disable/suspend the CP observation thread.
    void disable_cp_observation(void);

    /// @brief Enable/resume the CP observation thread.
    void enable_cp_observation(void);

    /// @brief Main function of the CP observation thread
    void cp_observation_worker(void);

    /// @brief Main function of the PP observation thread
    void pp_observation_worker(void);

    // ev@3370e4dd-95f4-47a9-aaec-ea76f34a66c9:v1
};

// ev@3d7da0ad-02c2-493d-9920-0bbbd56b9876:v1
// insert other definitions here
// ev@3d7da0ad-02c2-493d-9920-0bbbd56b9876:v1

} // namespace evse_board_support
} // namespace module

#endif // EVSE_BOARD_SUPPORT_EVSE_BOARD_SUPPORT_IMPL_HPP
