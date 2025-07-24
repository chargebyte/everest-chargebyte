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
#include <generated/types/cb_board_support.hpp>
#include <ra-utils/uart.h>
#include <ra-utils/cb_protocol.h>

using namespace std::chrono_literals;

///
/// Various helper for debug messages
///
std::ostream& operator<<(std::ostream& os, enum cc2_ccs_ready state);
std::ostream& operator<<(std::ostream& os, enum cs2_ce_state state);
std::ostream& operator<<(std::ostream& os, enum cs2_id_state state);
std::ostream& operator<<(std::ostream& os, enum cs2_estop_reason state);

///
/// A class for abstracting the safety processor UART interface on Charge SOM platform.
///
class CbParsley {

public:
    /// @brief Default constructor.
    CbParsley();

    /// @brief Destructor.
    ~CbParsley();

    /// @brief Resets the safety controller and opens the given UART.
    /// @param reset_gpio_line_name The name of the GPIO line to reset the safety processor.
    /// @param reset_active_low Flag whether the reset line has active-low polarity.
    /// @param serial_port The name of the UART device to use for communication with the safety processor.
    /// @param serial_trace Enable debug traces in communication library if set to true.
    void init(const std::string& reset_gpio_line_name, bool reset_active_low, const std::string& serial_port,
              bool serial_trace);

    /// @brief Releases the reset of the safety controller and establish communication,
    ///        i.e. if not yet done, retrieve firmware version etc.
    void enable();

    /// @brief Stop communication with safety controller and force the controller into reset.
    void disable();

    /// @brief Helper to indicate termination wish
    void terminate();

    /// @brief Resets the safety controller.
    void reset();

    /// @brief Allow switching from B0 to B
    void set_ccs_ready(bool enable);

    /// @brief Signal used to inform about ID state changes.
    ///        The parameter contains the new ID state.
    sigslot::signal<const enum cs2_id_state&> on_id_change;

    /// @brief Signal used to inform about CE state changes.
    ///        The parameter is the new CE state.
    sigslot::signal<const enum cs2_ce_state&> on_ce_change;

    /// @brief Signal used to inform about the reason for a stopped charging.
    ///        The parameter is the latest reason as reported by the safety controller.
    sigslot::signal<const enum cs2_estop_reason&> on_estop;

    /// @brief Return whether the safety controller detected an emergency state.
    bool is_emergency();

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
};
