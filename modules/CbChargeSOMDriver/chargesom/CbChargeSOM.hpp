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
#include <ra-utils/uart.h>
#include <ra-utils/cb_protocol.h>
/* FIXME needed? */
#include <generated/types/cb_board_support.hpp>

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

    /// @brief Resets the safety controller, opens the given UART and establish initial
    ///        communication with safety controller.
    /// @param reset_gpio_line_name The name of the GPIO line to reset the safety processor.
    /// @param reset_active_low Flag whether the reset line has active-low polarity.
    /// @param serial_port The name of the UART device to use for communication with the safety processor.
    /// @param is_pluggable Tells whether the safety processor needs to observe the proximity pilot.
    void init(const std::string& reset_gpio_line_name, bool reset_active_low, const std::string& serial_port,
              bool is_pluggable);

    /// @brief Helper to indicate termination wish
    void terminate();

    /// @brief Resets the safety controller.
    void reset();

    /// @brief Reads the current (cached) cable rating from safety controller.
    ///        In case the safety controller reported a value which cannot be mapped
    ///        we report 'None'. Error forwarding/reporting is done using the
    //         `on_pp_change` slot.
    /// @return A cable current rating using enum types::board_support_common::Ampacity
    types::board_support_common::Ampacity get_ampacity();

    /// @brief Signal used to inform about PP state changes.
    sigslot::signal<const enum pp_state> on_pp_change;

    /// @brief Allows the contactor to close or open it.
    /// @param allow_power_on True allows the contactor to close, false opens the contactor.
    void set_allow_power_on(bool allow_power_on);

    /// @brief Retrieves the number of supported temperature channels.
    /// @return The count of supported channels.
    unsigned int get_temperature_channels() const;

    /// @brief Returns whether the given temperature channel is enabled or not.
    /// @return True if the channel is enabled, false otherwise.
    bool is_temperature_enabled(unsigned int channel);

    /// @brief Returns whether the given temperature channel passed the internal selftest.
    /// @return True if the channel return valid measurements, false otherwise.
    bool is_temperature_valid(unsigned int channel);

    /// @brief Retrieves the current temperature measured on a given channel.
    /// @param channel The channel number
    /// @return The temperature in °C
    float get_temperature(unsigned int channel);

    /// @brief Return a firmware information string, i.e. version, application type, git hash...
    /// @return A string with the mentioned information.
    const std::string& get_fw_info() const;

private:
    /// @brief Holds the assembled firmware information string.
    std::string fw_info;

    /// @brief Remembers the serial port device name
    std::string serial_port;

    /// @brief The GPIO handle of the reset line for the safety controller.
    std::unique_ptr<gpiod::line_request> mcu_reset;

    /// @brief Time to hold the reset line active when resetting the safety controller.
    std::chrono::milliseconds mcu_reset_duration {5ms};

    /// @brief Helper to toggle the reset pin
    void set_mcu_reset(bool active);

    /// @brief Helper to send out the charge control frame
    void send_charge_control();

    /// @brief Helper to determine whether the received COM field is valid.
    bool is_valid_rx_com(enum cb_uart_com com);

    /// @brief Helper to request the given frame from safety controller
    void send_inquiry(enum cb_uart_com com);

    /// @brief Helper to request the given frame from safety controller
    ///        and to wait until the response was received.
    /// @return True, in case there was no response within a given timeout;
    ///         false otherwise.
    bool send_inquiry_and_wait(enum cb_uart_com com);

    /// @brief The UART context for libcbuart.
    struct uart_ctx uart;

    /// @brief The safety controller state context.
    struct safety_controller ctx;

    /// @brief Remember whether the system is with fixed cable or not.
    bool is_pluggable {false};

    /// @brief Thread for periodic transmitting UART frames
    std::thread tx_thread;

    /// @brief Prevents parallel sending of UART messages
    std::mutex tx_mutex;

    /// @brief Thread for receiving UART frames
    std::thread rx_thread;

    /// @brief Thread for processing notifications to higher layers.
    std::thread notify_thread;

    /// @brief FIXME
    std::vector<enum pp_state> pp_changes;

    /// @brief Mutex to ensure that only one inquiry request is in-flight at the same time
    std::mutex inquiry_mutex;

    /// @brief Condition variables used to wait for a specific UART frame
    std::vector<std::condition_variable> rx_cv = std::vector<std::condition_variable>(static_cast<std::size_t>(cb_uart_com::COM_MAX));

    /// @brief Used to protect the access to the individual data fields in `ctx`.
    std::vector<std::mutex> rx_mutex = std::vector<std::mutex>(static_cast<std::size_t>(cb_uart_com::COM_MAX));

    /// @brief Protects the `charge_control` field in `ctx`.
    std::mutex charge_control_mutex;

    /// @brief Helper to signal thread termination wish
    std::atomic_bool termination_requested {false};
};
