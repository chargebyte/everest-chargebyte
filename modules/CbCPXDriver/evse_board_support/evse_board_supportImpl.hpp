// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#ifndef EVSE_BOARD_SUPPORT_EVSE_BOARD_SUPPORT_IMPL_HPP
#define EVSE_BOARD_SUPPORT_EVSE_BOARD_SUPPORT_IMPL_HPP

//
// AUTO GENERATED - MARKED REGIONS WILL BE KEPT
// template version 3
//

#include "../CbCPXDriver.hpp"

// ev@75ac1216-19eb-4182-a85c-820f1fc2c091:v1
// insert your custom include headers here

#include <CPUtils.hpp>

struct cp_state_signal_side {
    /// @brief previous state is what we measured before the last round
    types::cb_board_support::CPState previous_state;

    /// @brief current state is what we measured in the last round
    types::cb_board_support::CPState current_state;

    /// @brief measured state is what we just measured in this round
    types::cb_board_support::CPState measured_state;

    /// @brief the voltage of the just completed measurement (in mV)
    int voltage;
};
// ev@75ac1216-19eb-4182-a85c-820f1fc2c091:v1

namespace module {
namespace evse_board_support {

struct Conf {};

class evse_board_supportImpl : public evse_board_supportImplBase {
public:
    evse_board_supportImpl() = delete;
    evse_board_supportImpl(Everest::ModuleAdapter* ev, const Everest::PtrContainer<CbCPXDriver>& mod, Conf& config) :
        evse_board_supportImplBase(ev, "evse_board_support"), mod(mod), config(config) {};

    // ev@8ea32d28-373f-4c90-ae5e-b4fcc74e2a61:v1
    // insert your public definitions here

    /// @brief Callback method called to react on new values of low-level PP observation
    void pp_observation_worker();

    /// @brief Callback method called to react on new values of low-level CP observation
    void cp_observation_worker();
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
    virtual void handle_ac_set_overcurrent_limit_A(double& value) override;

    // ev@d2d1847a-7b88-41dd-ad07-92785f06f5c4:v1
    // insert your protected definitions here
    // ev@d2d1847a-7b88-41dd-ad07-92785f06f5c4:v1

private:
    const Everest::PtrContainer<CbCPXDriver>& mod;
    const Conf& config;

    virtual void init() override;
    virtual void ready() override;

    // ev@3370e4dd-95f4-47a9-aaec-ea76f34a66c9:v1
    // insert your private definitions here

    /// @brief Hardware Capabilities
    types::evse_board_support::HardwareCapabilities hw_capabilities;

    /// @brief Mutex to enable/disable CP observation thread. Usually hold by the observation
    ///        worker thread but can be requested via `disable_cp_observation` method.
    std::mutex cp_observation_lock;

    /// @brief Tracks whether this EVSE is enabled or not.
    std::atomic_bool is_enabled {false};

    /// @brief Helper to track whether the CP observation is running
    bool cp_observation_enabled {false};

    /// @brief Tracks the last published CP state.
    std::atomic<types::cb_board_support::CPState> cp_current_state {types::cb_board_support::CPState::PowerOn};

    /// @brief Store CP state errors
    CPUtils::cp_state_errors cp_errors {};

    /// @brief Last published/detected ampacity
    types::board_support_common::ProximityPilot pp_ampacity {.ampacity = types::board_support_common::Ampacity::None};

    /// @brief Flag to remember whether we already published a proximity error
    std::atomic_bool pp_fault_reported {false};

    /// @brief Flag to remember whether we already published a control pilot error
    std::atomic_bool pilot_fault_reported {false};

    /// @brief Flag to remember whether we already published a diode error
    std::atomic_bool diode_fault_reported {false};

    /// @brief Flag to remember whether we already published a ventilation error
    std::atomic_bool ventilation_fault_reported {false};

    /// @brief Mutex to synchronize PP observation
    std::mutex pp_observation_lock;

    /// @brief Cached value for cp_observation_worker callback
    double previous_duty_cycle {100.0};

    /// @brief Cached values for cp_observation_worker callback
    struct cp_state_signal_side cp_positive_side {
        types::cb_board_support::CPState::PilotFault, types::cb_board_support::CPState::PilotFault,
            types::cb_board_support::CPState::PilotFault, 0
    };

    struct cp_state_signal_side cp_negative_side {
        types::cb_board_support::CPState::PilotFault, types::cb_board_support::CPState::PilotFault,
            types::cb_board_support::CPState::PilotFault, 0
    };

    /// @brief Mutex to protect `pp_ampacity` and `pp_fault_reported`
    std::mutex pp_mutex;

    /// @brief Mutex to protect `cp_errors` and `cp_current_state`.
    std::mutex cp_mutex;

    /// @brief Flag to remember whether we already published a contactor fault.
    std::atomic_bool contactor_fault_reported {false};

    /// @brief Helper to report contactor_fault
    void raise_contactor_error(const std::string& source, bool desired_state,
                               types::cb_board_support::ContactorState actual_state);

    // ev@3370e4dd-95f4-47a9-aaec-ea76f34a66c9:v1
};

// ev@3d7da0ad-02c2-493d-9920-0bbbd56b9876:v1
// insert other definitions here
// ev@3d7da0ad-02c2-493d-9920-0bbbd56b9876:v1

} // namespace evse_board_support
} // namespace module

#endif // EVSE_BOARD_SUPPORT_EVSE_BOARD_SUPPORT_IMPL_HPP
