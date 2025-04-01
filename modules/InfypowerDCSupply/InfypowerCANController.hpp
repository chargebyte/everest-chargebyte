// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>
#include <linux/can.h>
#include <linux/can/bcm.h>
#include <generated/types/power_supply_DC.hpp>
#include <sigslot/signal.hpp>
#include "InfypowerCANCmd.hpp"
#include "InfypowerCANID.hpp"

using namespace std::chrono_literals;

///
/// A class for abstracting the Infypower's CAN interface
///
class InfypowerCANController {

public:
    /// @brief Default constructor.
    InfypowerCANController();

    /// @brief Destructor.
    ~InfypowerCANController();

    /// @brief Open the given CAN interface and configure it with the desired bitrate.
    /// @param device The name of the CAN network device.
    /// @param bitrate The bitrate to use (in bit/s).
    /// @param can_source_address The source address to use in the CAN IDs.
    /// @param can_destination_address The destination address to use in the CAN IDs.
    /// @param dc_module_type String containing the power module type as configured by user.
    void init(const std::string& device, unsigned int bitrate, unsigned int can_source_address,
              unsigned int can_destination_address, const std::string& dc_module_type);

    /// @brief Switch between import (DC to grid) vs. export (grid to DC) mode.
    /// @param enable_import True, when the energy should flow from DC to grid.
    void set_import_mode(bool enable_import);

    /// @brief Enable or disable the DC power module.
    ///        This method first returns when the new state is reached. This is important
    ///        especially during enable true->false transition since EVerest's EVSEManager
    ///        disables the contactor directly after this call returns. This might damage
    ///        the contactors in case we did not yet disabled current flow.
    /// @param enable The desired new state.
    void set_enable(bool enable);

    /// @brief Set the desired values via CAN.
    /// @param voltage The desired new voltage (in [V])
    /// @param current The desired new current (in [A])
    void set_voltage_current(double voltage, double current);

    /// @brief Set the desired cut-off voltage via CAN.
    /// @param voltage The desired new voltage (in [V])
    void set_import_cutoff_voltage(double voltage);

    /// @brief Signal used to inform about voltage and current changes.
    ///        The callback receives two floats, the actual voltage and the current.
    sigslot::signal<const float&, const float&> on_vc_update;

    /// @brief Signal used to inform about errors.
    ///        The callback receives four parameters:
    ///        - a bool whether the error is actually present (true if so, false otherwise)
    ///        - a string used as the last part of Everest::error::ErrorType
    ///        - a string used as ErrorSubType
    ///        - a string used as error message
    sigslot::signal<bool, const std::string&, const std::string&, const std::string&> on_error;

    /// @brief Helper to remember the power modules capabilities in EVerest terms.
    types::power_supply_DC::Capabilities caps;

private:
    /// @brief Helper list which DC module types (refs configuration variable) are
    ///        actually bidirectional ones. This cannot be queried via CAN, so
    ///         user have to configure it.
    const std::vector<std::string> bidi_dc_module_types {
        "BEC",
        "BEG",
    };

    /// @brief Remember the CAN interface name
    std::string device;

    /// @brief Remembers which CAN source address to use (our controller side)
    unsigned char can_src_addr {InfypowerCANID::BROADCAST_ADDR};

    /// @brief Remembers which CAN destination address to use (DC module [group])
    unsigned char can_dst_addr {InfypowerCANID::BROADCAST_ADDR};

    /// @brief Remembers the user configured power module type
    std::string dc_module_type;

    /// @brief Helper to signal thread termination wish
    std::atomic_bool termination_requested {false};

    /// @brief File descriptor of the (opened) CAN BCM interface
    int can_bcm_fd {-1};

    /// @brief Helper to setup the CAN BCM
    void setup_can_bcm();

    /// @brief Helper for (looped) write to CAN BCM
    void push_to_can_bcm(const char* buf, size_t len, const std::string& err_msg);

    /// @brief CAN BCM receive thread
    std::thread can_bcm_thread;

    /// @brief CAN BCM receive thread worker
    void can_bcm_rx_worker();

    /// @brief File descriptor of the (opened) CAN RAW interface
    int can_raw_fd {-1};

    /// @brief Helper to setup the CAN RAW
    void setup_can_raw();

    /// @brief Helper for (looped) write to CAN RAW
    void push_to_can_raw(const struct can_frame* can_frame, const std::string& err_msg);

    /// @brief CAN RAW receive thread
    std::thread can_raw_thread;

    /// @brief CAN RAW receive thread worker
    void can_raw_rx_worker();

    /// @brief CAN ID used when pushing settings to power module
    InfypowerTxCANID can_id_write_cfg;

    /// @brief Helper to setup power module specifics, e.g. CAN frames etc.
    void pm_can_setup();

    /// @brief Remember the last requested voltage (in mV)
    int32_t requested_voltage {0};

    /// @brief Remember the last requested current (in mA)
    int32_t requested_current {0};

    /// @brief Remember the last received voltage (in mV)
    int32_t received_voltage {0};

    /// @brief Remember the last received current (in mA)
    int32_t received_current {0};

    /// @brief Remember the last requested cut-off voltage (in mV)
    int32_t requested_cutoff_voltage {0};

    /// @brief The time in milliseconds until a feedback CAN frame is expected
    ///        for actions triggered by us.
    std::chrono::milliseconds request_timeout {1s};

    /// @brief List of "commands in flight" which are caused by the BCM part.
    ///        This vector holds only pointers to the real instances.
    ///        Of these instances, the callback mechanism is used, instead
    ///        of the condition variable stuff.
    std::vector<std::unique_ptr<InfypowerCANCmd>> can_bcm_cmds;

    /// @brief List of "commands in flight", i.e. commands we have sent and expecting
    ///        CAN frames as feedback.
    std::vector<std::reference_wrapper<InfypowerCANCmd>> expected_cmds;

    /// @brief Protects `expected_cmds`
    std::mutex expected_cmds_mutex;

    /// @brief Helper to add a command to the list
    void register_expected_cmd(InfypowerCANCmd& cmd);

    /// @brief Helper to remove a command from the list
    void unregister_expected_cmd(InfypowerCANCmd& cmd);

    /// @brief Helper to process all steps for a single command
    ///        Return true in case of timeout, false otherwise.
    bool process_cmd(InfypowerCANCmd& cmd);

    /// @brief Remember how many modules we found in the power module group
    unsigned int pm_count {0};

    /// @brief Helper to raise an exception when CAN feedback is incomplete
    void raise_incomplete_feedback(const InfypowerCANCmd& cmd);

    /// @brief Query the power module group master for the count of modules in this group.
    void pm_query_count();

    /// @brief Query the power module capabilities.
    void pm_query_caps();
};
