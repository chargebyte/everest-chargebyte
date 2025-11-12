// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once
#include <atomic>
#include <condition_variable>
#include <queue>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <gpiod.hpp>
#include <sigslot/signal.hpp>
#include <generated/types/board_support_common.hpp>
// B0 is defined in terminios.h for UART baudrate, but in CEState for MCS too - so undefine it before the inclusion
#undef B0
#include <generated/types/cb_board_support.hpp>
#include <ra-utils/uart.h>
#include <ra-utils/cb_protocol.h>

using namespace std::chrono_literals;

///
/// Various helper for debug messages
///
std::ostream& operator<<(std::ostream& os, enum cp_state state);
std::ostream& operator<<(std::ostream& os, enum pp_state state);
std::ostream& operator<<(std::ostream& os, enum contactor_state state);
std::ostream& operator<<(std::ostream& os, enum estop_state state);
std::ostream& operator<<(std::ostream& os, enum cs1_safestate_reason reason);
std::ostream& operator<<(std::ostream& os, enum cs_safestate_active state);

///
/// A class for abstracting the safety processor UART interface on Charge SOM platform.
///
class CbChargeSOM {

public:
    /// @brief Default constructor.
    CbChargeSOM();

    /// @brief Destructor.
    ~CbChargeSOM();

    /// @brief Resets the safety controller and opens the given UART.
    /// @param reset_gpio_line_name The name of the GPIO line to reset the safety processor.
    /// @param reset_active_low Flag whether the reset line has active-low polarity.
    /// @param serial_port The name of the UART device to use for communication with the safety processor.
    /// @param is_pluggable Tells whether the safety processor needs to observe the proximity pilot.
    /// @param serial_trace Enable debug traces in communication library if set to true.
    void init(const std::string& reset_gpio_line_name, bool reset_active_low, const std::string& serial_port,
              bool is_pluggable, bool serial_trace);

    /// @brief Releases the reset of the safety controller and establish communication,
    ///        i.e. if not yet done, retrieve firmware version etc.
    void enable();

    /// @brief Stop communication with safety controller and force the controller into reset.
    void disable();

    /// @brief Helper to indicate termination wish
    void terminate();

    /// @brief Resets the safety controller.
    void reset();

    /// @brief Signal used to inform about PP state changes.
    ///        The parameter contains the new ampacity.
    sigslot::signal<const types::board_support_common::Ampacity&> on_pp_change;

    /// @brief Signal used to inform about PP (measurement) errors.
    ///        The parameter contains an error reason/description.
    sigslot::signal<const std::string&> on_pp_error;

    /// @brief Signal used to inform about CP state changes.
    ///        The parameter is the new CP state.
    sigslot::signal<const types::cb_board_support::CPState&> on_cp_change;

    /// @brief Signal used to inform about various errors, e.g. CP Short Circuits,
    ///        Diode Faults and contactor errors.
    ///        Callee is expected to check in detail.
    sigslot::signal<> on_cp_error;

    /// @brief Signal used to inform about the reason for a stopped charging.
    ///        The parameter is the latest reason as reported by the safety controller.
    sigslot::signal<const enum cs1_safestate_reason&> on_estop;

    /// @brief Signal used to inform about changed safe state
    ///        The parameter is the latest state as reported by the safety controller.
    sigslot::signal<const enum cs_safestate_active&> on_safestate_active;

    /// @brief Signal emitted whenever a contactor switch is detected.
    ///        First parameter is the name of the contactor, second parameter is actual state.
    sigslot::signal<const std::string&, types::cb_board_support::ContactorState> on_contactor_change;

    /// @brief Signal used to inform about errors during contactor switching.
    ///        First parameter is the name of the contactor, second parameter is intended state
    ///        and third parameter is actual state.
    sigslot::signal<const std::string&, bool, types::cb_board_support::ContactorState> on_contactor_error;

    /// @brief Signal emitted whenever an error message is received.
    ///        The parameters are filled with the data from the latest error message.
    sigslot::signal<bool, unsigned int, const std::string&, unsigned int, const std::string&, unsigned int,
                    unsigned int>
        on_errmsg;

    /// @brief Return whether the safety controller detected an emergency state.
    bool is_emergency();

    /// @brief Set a new duty cycle.
    /// @param duty_cycle The desired duty cycle in percent [0.1 %].
    void set_duty_cycle(unsigned int duty_cycle);

    /// @brief Get the current/actual duty cycle in [0.1 %].
    unsigned int get_duty_cycle();

    /// @brief Get the current state of the diode fault signal
    bool get_diode_fault();

    /// @brief Get the current state of the CP short circuit signal
    bool get_cp_short_circuit();

    /// @brief Closes the contactor (on = true), or opens it (on = false):
    ///        This is a synchronous call, i.e. it waits until it is confirmed
    ///        by feedback signal (if used).
    /// @return True on success, false on error.
    bool switch_state(bool on);

    /// @brief Return the current contactor state (even when no contactor is configured)
    bool get_contactor_state();

    /// @brief Remember whether the PT1000 State frame was received at least once.
    bool temperature_data_is_valid {false};

    /// @brief Retrieves the number of supported temperature channels.
    /// @return The count of supported channels.
    unsigned int get_temperature_channels() const;

    /// @brief Returns whether the given temperature channel is enabled or not.
    /// @return True if the channel is enabled, false otherwise.
    bool is_temperature_enabled(unsigned int channel);

    /// @brief Returns whether the given temperature channel passed the internal selftest.
    /// @return True if the channel return valid measurements, false otherwise.
    bool is_temperature_valid(unsigned int channel);

    /// @brief Returns the temperature channel error flags
    /// @return The flags for this channel
    unsigned int get_temperature_errors(unsigned int channel);

    /// @brief Retrieves the current temperature measured on a given channel.
    /// @param channel The channel number
    /// @return The temperature in °C
    float get_temperature(unsigned int channel);

    /// @brief Return a firmware information string, i.e. version, application type, git hash...
    ///        The return value is filled once the communication with the safety controller is
    ///        was started, but not before.
    /// @return A string with the mentioned information.
    const std::string& get_fw_info() const;

private:
    /// @brief Remember whether the system is with fixed cable or not.
    bool is_pluggable {false};

    /// @brief Remembers the serial port device name
    std::string serial_port;

    /// @brief The UART context for libcbuart.
    struct uart_ctx uart;

    /// @brief The safety controller state context.
    struct safety_controller ctx;

    /// @brief The GPIO handle of the reset line for the safety controller.
    std::unique_ptr<gpiod::line_request> mcu_reset;

    /// @brief Time to hold the reset line active when resetting the safety controller.
    std::chrono::milliseconds mcu_reset_duration {5ms};

    /// @brief Holds the assembled firmware information string.
    std::string fw_info;

    /// @brief Remember whether EvseManager enabled this port.
    std::atomic_bool evse_enabled {false};

    /// @brief Flag to control sending of Charge Control messages
    std::atomic_bool tx_cc_enabled {false};

    /// @brief Flag whether we should expect frames from the safety controller.
    std::atomic_bool rx_enabled {false};

    /// @brief Thread for periodic transmitting UART frames
    std::thread tx_thread;

    /// @brief Prevents parallel sending of UART messages
    std::mutex tx_mutex;

    /// @brief Thread for receiving UART frames
    std::thread rx_thread;

    /// @brief Thread for processing notifications to higher layers.
    std::thread notify_thread;

    /// @brief Mutex to protect access to the `charge_state_changes` queue.
    std::mutex notify_mutex;

    /// @brief Condition variables used to wait for updates on `charge_state_changes`
    std::condition_variable notify_cv;

    /// @brief Queue used to serialize changes of Charge State frame for notifying
    std::queue<uint64_t> charge_state_changes;

    /// @brief Thread for pushing received error messages to higher layers.
    std::thread errmsg_thread;

    /// @brief Mutex to protect access to the `error_messages` queue.
    std::mutex errmsg_mutex;

    /// @brief Condition variables used to wait for updates on `error_messages`
    std::condition_variable errmsg_cv;

    /// @brief Queue used to serialize Error Message frames for notifying
    std::queue<uint64_t> errmsg_queue;

    /// @brief Mutex to ensure that only one inquiry request is in-flight at the same time
    std::mutex inquiry_mutex;

    /// @brief Condition variables used to wait for a specific UART frame and/or update to the field in `ctx`
    std::vector<std::condition_variable> rx_cv =
        std::vector<std::condition_variable>(static_cast<std::size_t>(cb_uart_com::COM_MAX));

    /// @brief Used to protect the access to the individual data fields in `ctx`.
    std::vector<std::mutex> ctx_mutexes = std::vector<std::mutex>(static_cast<std::size_t>(cb_uart_com::COM_MAX));

    /// @brief Helper to signal thread termination wish
    std::atomic_bool termination_requested {false};

    /// @brief Helper to track the current MCU reset state
    ///        Background: using the GPIO line itself is heavy load due to call into kernel etc.
    ///                    and it is available only after the GPIO was requested. But we launch
    ///                    some threads before.
    bool is_mcu_reset_active {true};

    /// @brief Helper to toggle the reset pin
    void set_mcu_reset(bool active);

    /// @brief Helper to determine whether the received COM field is invalid (or better: not understood on our side).
    bool is_unexpected_rx_com(enum cb_uart_com com);

    /// @brief Helper to send out the charge control frame
    void send_charge_control();

    /// @brief Helper to request the given frame from safety controller
    void send_inquiry(enum cb_uart_com com);

    /// @brief Helper to request the given frame from safety controller
    ///        and to wait until the response was received.
    /// @return True, in case there was no response within a given timeout;
    ///         false otherwise.
    bool send_inquiry_and_wait(enum cb_uart_com com);

    /// @brief Internal helper to determine the current contactor state.
    bool get_contactor_state_no_lock();

    /// @brief Helper to map the internal PP enum to the EVerest type system.
    ///        A 'std::runtime_error` is raised in case the mapping fails, e.g.
    ///        when Type 1 related states are found.
    /// @return A cable current rating using `types::board_support_common::Ampacity`
    types::board_support_common::Ampacity pp_state_to_ampacity(enum pp_state pp_state);
};
