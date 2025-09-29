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
#include <net/if.h>
#include <sigslot/signal.hpp>
#include <generated/types/board_support_common.hpp>
#include <generated/types/cb_board_support.hpp>
#include "can_interface/can.h"
#include <generated/interfaces/cb_cpx_temperatures/Implementation.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <linux/can/bcm.h>
// #include "../CbCpxDriver.hpp"

using namespace std::chrono_literals;

/* maximum supported contactors */
#define CB_PROTO_MAX_CONTACTORS 2

/* maximum supported estops */
#define CB_PROTO_MAX_ESTOPS 3

/* maximum count of PT1000 channels */
#define CB_PROTO_MAX_PT1000S 4

// scale of measured temperatures
#define CB_PROTO_TEMP_SCALE 0.1

typedef struct {
    struct can_charge_state1_t charge_state;
    struct can_charge_control1_t charge_control;
    struct can_pt1000_state_t pt1000_state;
    struct can_firmware_version_t firmware_version;
    struct can_git_hash_t git_hash;
    struct can_inquiry_packet_t inquiry_packet;
} com_data_t;

namespace module {
    class Conf;
}

///
/// A class for abstracting the safety processor UART interface on Charge SOM platform.
///
class CbCpx {

public:
    /// @brief Default constructor.
    CbCpx(const module::Conf& config);

    /// @brief Destructor.
    ~CbCpx();

    /// @brief Open the given UART and establish initial communication with safety controller.
    /// @param is_pluggable Tells whether the safety processor needs to observe the proximity pilot.
    void init(bool is_pluggable);

    /// @brief Releases the reset of the safety controller and establish communication,
    ///        i.e. if not yet done, retrieve firmware version etc.
    void enable();

    /// @brief Signal used to inform when firmware information string was assembled
    ///        The parameter contains the new content
    sigslot::signal<const std::string&> on_fw_info;

    /// @brief Helper to indicate termination wish
    void terminate();

    /// @brief Helper to map the internal PP enum to the EVerest type system.
    ///        A 'std::runtime_error` is raised in case the mappig fails, e.g.
    ///        when Type 1 related states are found.
    /// @return A cable current rating using `types::board_support_common::Ampacity`
    types::board_support_common::Ampacity pp_state_to_ampacity(uint8_t pp_state);
    
    /// @brief Reads the current (cached) cable rating from the safety controller.
    ///        It uses `pp_state_to_ampacity`, in other words it raises an exception
    ///        in case the value cannot be mapped.
    /// @return A cable current rating using `types::board_support_common::Ampacity`
    types::board_support_common::Ampacity get_ampacity();


    /// @brief Initialize Charge Control message in CAN BCM.
    void charge_control_init();

    /// @brief Update Charge Control message in CAN BCM.
    void charge_control_update();

    /// @brief Return whether the safety controller detected an emergency state.
    bool is_emergency();

    /// @brief Set a new duty cycle.
    /// @param duty_cycle The desired duty cycle in percent [0.1 %].
    void set_duty_cycle(unsigned int duty_cycle);
    
    /// @brief Set contactor state.
    /// @param on Desired contactor state.
    /// @return 0 if CPX switched to desired state, 1 or 2 depending on which contactor did not switch
    int switch_state(bool on);

    /// @brief Get current CP state.
    uint8_t get_cp_state();

    /// @brief Function to call all Charge State getter-functions
    /// @param data Received CAN data
    void read_charge_state(uint8_t* data);

    /// @brief Function to call all PT1000 State getter-functions
    /// @param data Received CAN data
    void read_pt1000_state(uint8_t* data);

    /// @brief Function to read Firmware Version info
    /// @param data Received CAN data
    void read_fw_version(uint8_t* data);

    /// @brief Function to read Git Hash info
    /// @param data Received CAN data
    void read_git_hash(uint8_t* data);

    /// @brief Return the current contactor state (even when no contactor is configured)
    bool get_contactor_state();

    /// @brief Get the current/actual duty cycle in [0.1 %].
    unsigned int get_duty_cycle();

    /// @brief Signal used to inform about PP state changes.
    ///        The parameter contains the new PP state.
    sigslot::signal<uint8_t> on_pp_change;

    /// @brief Signal used to inform about CP state changes.
    ///        The parameter contains the new CP state.
    sigslot::signal<uint8_t> on_cp_change;

    /// @brief Signal used to inform about various errors, e.g. CP Short Circuits,
    ///        Diode Faults and contactor errors.
    ///        Callee is expected to check in detail.
    sigslot::signal<> on_cp_error;

    /// @brief Signal used to inform about errors during contactor switching.
    sigslot::signal<const std::string&, bool, types::cb_board_support::ContactorState> on_contactor_error;

    /// @brief Signal used to inform about a charging stop caused by ESTOP signal.
    ///        The first parameter is the number of the ESTOP signal which changed,
    ///        the second parameter tells whether the signal is active.
    sigslot::signal<const unsigned int&, const bool&> on_estop;

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

    /// @brief Returns whether the given temperature channel passed the internal selftest.
    /// @return True if the channel return valid measurements, false otherwise.
    bool is_pt_selftest_failed(unsigned int channel);

    /// @brief Returns whether the given temperature channel passed the internal selftest.
    /// @return True if the channel return valid measurements, false otherwise.
    bool is_pt_charging_stopped(unsigned int channel);

    /// @brief Retrieves the current temperature measured on a given channel.
    /// @param channel The channel number
    /// @return The temperature in °C
    float get_temperature(unsigned int channel);

    // /// @brief Signal used to share temperatures of PT1000 sensors.
    // sigslot::signal<const std::vector<types::temperature::Temperature>&> temperature;

    /// @brief Helper to signal thread termination wish
    std::atomic_bool termination_requested {false};

    /// @brief
    uint8_t get_cs_diode_fault();

    /// @brief
    uint8_t get_cs_short_circuit();

    /// @brief Signal used to inform about CPX timeout.
    ///        The parameter contains the ID of timed out CPX.
    sigslot::signal<bool> on_cpx_timeout;

private:
    /// @brief Reference to the CPX-config provided by manifest.yaml
    const module::Conf& config;

    /// @brief Flag to control sending of Charge Control messages
    std::atomic_bool tx_cc_enabled {false};

    /// @brief Flag to tell we are expecting incoming BCM messages
    std::atomic_bool rx_bcm_enabled {false};

    /// @brief Flag to tell we are expecting incoming RAW messages
    std::atomic_bool rx_raw_enabled {false};

    /// @brief Remember whether the system is with fixed cable or not.
    bool is_pluggable {false};

    /// @brief Struct of CAN Socket Address
    struct sockaddr_can addr;
    
    /// @brief Interface request structure
    struct ifreq ifr;

    /// @brief File descriptor of the (opened) CAN RAW interface
    int can_raw_fd {-1};

    /// @brief File descriptor of the (opened) CAN BCM RX interface
    int can_bcm_rx_fd {-1};

    /// @brief File descriptor of the (opened) CAN BCM TX interface
    int can_bcm_tx_fd {-1};

    /// @brief Holds the assembled firmware information string.
    std::string fw_info;

    /// @brief Thread for receiving CAN BCM messages
    std::thread can_bcm_rx_thread;

    /// @brief Thread for receiving CAN RAW messages
    std::thread can_raw_rx_thread;

    /// @brief Thread for notifications upon changes
    std::thread notify_thread;

    // /// @brief Thread for publishing PT1000 information
    // std::thread publish_thread;

    /// @brief Register BCM rx messages
    void can_bcm_rx_init();

    /// @brief CAN BCM receive thread worker
    void can_bcm_rx_worker();

    /// @brief CAN RAW receive thread worker
    void can_raw_rx_worker();

    /// @brief Notify thread worker
    void notify_worker();

    // /// @brief Notify thread worker
    // void publish_worker();

    /// @brief struct holding all available messages
    com_data_t com_data{};

    /// @brief Prevents parallel access to charge control info
    std::mutex cc_mutex;

    /// @brief Prevents parallel access to charge state info
    std::mutex cs_mutex;

    /// @brief Prevents parallel access to pt1000 info
    std::mutex pt_mutex;

    /// @brief Prevents parallel access to firmware version info
    std::mutex fv_mutex;

    /// @brief Prevents parallel access to git hash info
    std::mutex gh_mutex;

    /// @brief Prevents parallel access to inquiry packet info
    std::mutex ip_mutex;

    /// @brief Prevents parallel access to notifications
    std::mutex  notify_mutex;

    /// @brief Prevents parallel access to BCM socket
    std::mutex  bcm_mutex;

    /// @brief Save last new received Charge State message
    struct {
        struct bcm_msg_head msg_head;
        struct can_frame frame;
    } cs_msg;

    /// @brief Save last new received PT1000 State message
    struct {
        struct bcm_msg_head msg_head;
        struct can_frame frame;
    } pt_msg;

    /// @brief Condition variable used to wait for Charge State message
    std::condition_variable rx_cs_cv;

    /// @brief Condition variable used to wait for PT1000 State message
    std::condition_variable rx_pt_cv;

    /// @brief Condition variable used to wait for Firmware Version message
    std::condition_variable rx_fv_cv;

    /// @brief Condition variable used to wait for Git Hash message
    std::condition_variable rx_gh_cv;

    /// @brief Condition variable used to wait for changes relevant for notifications
    std::condition_variable notify_cv;

    /// @brief Remember whether EvseManager enabled this port.
    std::atomic_bool evse_enabled {false};

    /// @brief Remember if Charge State updates were received.
    std::atomic_bool cs_updated {false};

    // /// @brief Helper to track the current MCU reset state
    // ///        Background: using the GPIO line itself is heavy load due to call into kernel etc.
    // ///                    and it is available only after the GPIO was requested. But we launch
    // ///                    some threads before.
    // bool is_mcu_reset_active {true};

    // /// @brief Helper to toggle the reset pin
    // void set_mcu_reset(bool active);

    /// @brief
    uint16_t get_cs_current_duty_cycle();

    /// @brief
    uint8_t get_cs_pwm_active();

    /// @brief
    uint8_t get_cs_current_cp_state();

    /// @brief
    uint8_t get_cs_current_pp_state();

    /// @brief
    uint8_t get_cs_contactor_state(int contactor);

    /// @brief
    uint8_t get_cc_contactor_state(int contactor);

    /// @brief
    bool is_cs_contactor_error(int contactor);

    /// @brief
    uint8_t get_cs_hv_ready();

    /// @brief
    bool is_cs_estop_charging_abort(int estop);

    /// @brief
    bool get_pt1000_is_active(int channel);

    /// @brief Internal helper to determine the current contactor state.
    bool get_contactor_state_no_lock();

    /// @brief Internal helper to determine CAN-ID in accordance with CPX-ID.
    canid_t get_can_id(int cpx_id, u_int message_id);

    bool print_can_id_info {false};

    /// @brief Remember if CPX CAN-message timed out
    std::atomic<bool> bcm_rx_timeout {false};

    /// @brief Helper to request and save firmware version and git hash
    void get_firmware_and_git_hash();

    /// @brief Remember if CPX was ever connected after EVerest start up
    std::atomic<bool> has_cpx_connected_once {false};
};
