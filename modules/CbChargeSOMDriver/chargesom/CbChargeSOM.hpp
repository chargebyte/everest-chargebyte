// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once
#include <chrono>
#include <mutex>
#include <string>
#include <sigslot/signal.hpp>
#include <generated/types/board_support_common.hpp>
#include <generated/types/cb_board_support.hpp>
#include <uart.h>
#include <cb_protocol.h>

using namespace std::chrono_literals;

///
/// A class for abstracting the safety processor UART interface on Charge SOM platform.
///
class CbChargeSOM {

public:
    /// @brief Default constructor.
    CbChargeSOM();

    /// @brief Destructor.
    ~CbChargeSOM();

    /// @brief Open the given UART and establish initial communication with safety controller.
    /// @param serial_port The name of the UART device to use for communication with the safety processor.
    /// @param is_pluggable Tells whether the safety processor needs to observe the proximity pilot.
    void init(const std::string& serial_port, bool is_pluggable);

    // @brief Return the value of recovery_delay.
    std::chrono::milliseconds get_recovery_delay_ms();

    /// @brief Read the current Control Pilot voltage values, i.e. both signal sides.
    /// @param positive_value Reference which will be updated with the current value of the positive side (in mV).
    /// @param negative_value Reference which will be updated with the current value of the negative side (in mV).
    void cp_get_values(int& positive_value, int& negative_value);

    /// @brief Helper to map a measured voltage to a CP state (takes hysteresis into account)
    types::cb_board_support::CPState cp_voltage_to_state(int voltage,
                                                         types::cb_board_support::CPState previous_state) const;

    /// @brief Get the current duty cycle in percent.
    /// @return The current duty cycle.
    double cp_get_duty_cycle() const;

    /// @brief Set a new duty cycle.
    /// @param duty_cycle The desired duty cycle in percent.
    void cp_set_duty_cycle(double duty_cycle);

    /// @brief Check whether the current duty cycle is nominal.
    /// @return True when configured duty cycle is >0 and <100% (nominal duty cycle),
    ///         false otherwise.
    bool cp_is_nominal_duty_cycle() const;

    /// @brief Check whether the PWM output is actively driven.
    /// @return True when a signal is driven, false otherwise.
    bool cp_is_enabled();

    /// @brief Disable the pin output. This drives 100% for a short time to ensure a
    ///        deterministic falling edge on the signal. Then the PWM is disabled.
    void cp_disable();

    /// @brief Reads the current (cached) cable rating from safety controller.
    ///        Throws a `std::underflow_error`exception in case the ADC reading is out of
    ///        the expected range and cannot be mapped to a result value.
    /// @param[out] voltage The physically measured voltage at the ADC input pin in mV
    /// @return A cable current rating using enum types::board_support_common::Ampacity
    types::board_support_common::Ampacity get_ampacity(int& voltage);

    /// @brief Allows the contactor to close or open it.
    /// @param allow_power_on True allows the contactor to close, false opens the contactor.
    void set_allow_power_on(bool allow_power_on);

    /// @brief Performs a full communication exchange with the safety controller, i.e.
    ///        all values are read back and our desired values are communicated to it.
    ///        In case of error, it throws a `std::system_error` with the encountered errno.
    ///        This high-level/public function ensures that `sc_mutex` is hold.
    void sync();

    /// @brief Signal used to inform about changed PP state.
    sigslot::signal<> signal_pp_state_change;

    /// @brief Signal used to inform about changed CP state.
    sigslot::signal<> signal_cp_state_change;

private:
    /// @brief The context for libcbuart to operate on.
    struct safety_ctx ctx;

    /// @brief Mutex to protect access to `ctx.data'.
    ///        Note: It is expected that `ctx.data` is always kept in sync with the safety controller, so this mutex must also
    ///              be held, when UART communication is done.
    std::mutex ctx_mutex;

    /// @brief Remember whether the system is with fixed cable or not.
    bool is_pluggable {false};

    /// @brief The recovery delay is an enforced time after an error occurred,
    ///        which is given to the safety controller without any attempt of UART communication.
    std::chrono::milliseconds recovery_delay_ms {100ms};

    /// @brief Performs a full communication exchange with the safety controller, i.e.
    ///        all values are read back and our desired values are communicated to it.
    ///        In case of error, it throws a `std::system_error` with the encountered errno.
    ///        This is the low-level function which assumes that `ctx_mutex` is already held by caller.
    void sync_with_hw();
};
