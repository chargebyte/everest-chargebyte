// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <array>
#include <stdexcept>
#include <string>
#include <sstream>
#include <system_error>
#include <thread>
#include <linux/can.h>
#include <linux/can/bcm.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include "CbCpx.hpp"
#include "../CbCpxDriver.hpp"
#include <everest/logging.hpp>
#include <generated/types/cb_board_support.hpp>
#include <generated/interfaces/cb_cpx_temperatures/Implementation.hpp>

using namespace std::chrono_literals;

CbCpx::CbCpx(const module::Conf& config) : config(config) {
    // we have to convert can_interface of type std::string to const char *
    const char* can_interface_cstr = this->config.can_interface.c_str();
    
    // open a CAN BCM RX socket, bound to the desired interface
    this->can_bcm_rx_fd = socket(PF_CAN, SOCK_DGRAM, CAN_BCM);
    if (this->can_bcm_rx_fd == -1)
        throw std::system_error(errno, std::generic_category(), "socket(PF_CAN, CAN_BCM) failed");

    strncpy(ifr.ifr_name, can_interface_cstr, sizeof(ifr.ifr_name));
    if (ioctl(this->can_bcm_rx_fd, SIOCGIFINDEX, &ifr))
        throw std::system_error(errno, std::generic_category(),
                                "Couldn't determine interface number of " + this->config.can_interface);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (connect(this->can_bcm_rx_fd, (struct sockaddr*)&addr, sizeof(addr)))
        throw std::system_error(errno, std::generic_category(), "Couldn't connect CAN BCM socket on " + this->config.can_interface);

    // open a CAN BCM RX socket, bound to the desired interface
    this->can_bcm_tx_fd = socket(PF_CAN, SOCK_DGRAM, CAN_BCM);
    if (this->can_bcm_tx_fd == -1)
        throw std::system_error(errno, std::generic_category(), "socket(PF_CAN, CAN_BCM) failed");

    strncpy(ifr.ifr_name, can_interface_cstr, sizeof(ifr.ifr_name));
    if (ioctl(this->can_bcm_tx_fd, SIOCGIFINDEX, &ifr))
        throw std::system_error(errno, std::generic_category(),
                                "Couldn't determine interface number of " + this->config.can_interface);

    if (connect(this->can_bcm_tx_fd, (struct sockaddr*)&addr, sizeof(addr)))
        throw std::system_error(errno, std::generic_category(), "Couldn't connect CAN BCM socket on " + this->config.can_interface);

    // open a CAN RAW socket, bound to the desired interface
    this->can_raw_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (this->can_raw_fd == -1)
        throw std::system_error(errno, std::generic_category(), "socket(PF_CAN, CAN_RAW) failed");

    // set interface name
    std::strncpy(ifr.ifr_name, can_interface_cstr, IFNAMSIZ - 1);
    if (ioctl(this->can_raw_fd, SIOCGIFINDEX, &ifr) < 0) {
        throw std::runtime_error("Failed to get interface index on " + this->config.can_interface);
    }

    // set CAN filters for RAW socket
    struct can_filter filters[2];
    filters[0].can_id   = get_can_id(this->config.id, CAN_FIRMWARE_VERSION_FRAME_ID);
    filters[0].can_mask = CAN_EFF_FLAG | CAN_EFF_MASK;
    filters[1].can_id   = get_can_id(this->config.id, CAN_GIT_HASH_FRAME_ID);
    filters[1].can_mask = CAN_EFF_FLAG | CAN_EFF_MASK;

    if (setsockopt(this->can_raw_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filters, sizeof(filters)) < 0) {
        throw std::system_error(errno, std::generic_category(), "setsockopt(CAN_RAW_FILTER) failed");
    }

    // bind socket
    if (bind(this->can_raw_fd, (struct sockaddr*)&addr, sizeof(addr)))
        throw std::system_error(errno, std::generic_category(), "Couldn't bind CAN RAW socket!");

    // setup BCM rx messages
    this->can_bcm_rx_init();

    // launch notify thread
    this->notify_thread = std::thread(&CbCpx::notify_worker, this);

    // launch CAN BCM rx thread
    this->can_bcm_rx_thread = std::thread(&CbCpx::can_bcm_rx_worker, this);

    // launch CAN RAW rx thread
    this->can_raw_rx_thread = std::thread(&CbCpx::can_raw_rx_worker, this);
}

CbCpx::~CbCpx() {
    this->terminate();
}

void CbCpx::init(bool is_pluggable) {
    // remember this setting
    this->is_pluggable = is_pluggable;
}

void CbCpx::terminate() {
    this->evse_enabled = false;
    this->tx_cc_enabled = false;
    this->rx_bcm_enabled = false;
    this->rx_raw_enabled = false;

    this->termination_requested = true;
    this->notify_worker_cv.notify_all();

    this->duty_cycle_check_termination_requested = true;
    this->duty_cycle_check_cv.notify_all();
    
    this->timeout_watchdog_termination_requested = true;
    this->timeout_watchdog_cv.notify_all();

    // close CAN BCM RX socket
    if (this->can_bcm_rx_fd >= 0) {
        close(this->can_bcm_rx_fd);
        this->can_bcm_rx_fd = -1;
    }

    // close CAN BCM TX socket
    if (this->can_bcm_tx_fd >= 0) {
        close(this->can_bcm_tx_fd);
        this->can_bcm_tx_fd = -1;
    }

    // close CAN RAW socket
    if (this->can_raw_fd >= 0) {
        close(this->can_raw_fd);
        this->can_raw_fd = -1;
    }

    if (this->notify_thread.joinable()) {
        this->notify_thread.join();
    }

    if (this->can_bcm_rx_thread.joinable()) {
        this->can_bcm_rx_thread.join();
    }

    if (this->can_raw_rx_thread.joinable()) {
        this->can_raw_rx_thread.join();
    }

    if (this->duty_cycle_check_thread.joinable()) {
        this->duty_cycle_check_thread.join();
    }

    if (this->timeout_watchdog_thread.joinable()) {
        this->timeout_watchdog_thread.join();
    }
}

void CbCpx::get_firmware_and_git_hash() {
    // save current firmware information
    std::unique_lock<std::mutex> fv_lock(this->fv_mutex);
    std::unique_lock<std::mutex> gh_lock(this->gh_mutex);

    struct can_firmware_version_t current_firmware_version_info = com_data.firmware_version;
    struct can_git_hash_t current_git_hash_info = com_data.git_hash;
        
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    uint8_t payload[8];

    this->print_can_id_info = true;

    // frame information valid for firmware version and git hash
    frame.can_id = get_can_id(this->config.id, CAN_INQUIRY_PACKET_FRAME_ID);
    frame.can_dlc = CAN_INQUIRY_PACKET_LENGTH;
        
    // query firmware version
    com_data.inquiry_packet.packet_id = CAN_INQUIRY_PACKET_PACKET_ID_FIRMWARE_VERSION_CHOICE;
    can_inquiry_packet_pack(payload, &com_data.inquiry_packet, CAN_INQUIRY_PACKET_LENGTH);
    memcpy(frame.data, payload, CAN_INQUIRY_PACKET_LENGTH);

    fv_lock.unlock();

    int bytes_sent = write(can_raw_fd, &frame, sizeof(frame));
    if (bytes_sent != sizeof(frame)) {
        throw std::runtime_error("Could not request firmware version!");
    }

    // we should have a response within 1 second
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // query git hash
    com_data.inquiry_packet.packet_id = CAN_INQUIRY_PACKET_PACKET_ID_GIT_HASH_CHOICE;
    can_inquiry_packet_pack(payload, &com_data.inquiry_packet, CAN_INQUIRY_PACKET_LENGTH);
    memcpy(frame.data, payload, CAN_INQUIRY_PACKET_LENGTH);

    gh_lock.unlock();

    bytes_sent = write(can_raw_fd, &frame, sizeof(frame));
    if (bytes_sent != sizeof(frame)) {
        throw std::runtime_error("Could not request git hash!");
    }

    // we should have a response within 1 second
    std::this_thread::sleep_for(std::chrono::seconds(1));

    this->print_can_id_info = false;

    // pack firmware info
    fv_lock.lock();
    gh_lock.lock();

    // check if firmware info was really received on RAW thread
    if ((current_firmware_version_info.major_version == com_data.firmware_version.major_version) && 
        (current_firmware_version_info.minor_version == com_data.firmware_version.minor_version) &&
        (current_firmware_version_info.build_version == com_data.firmware_version.build_version) &&
        (current_firmware_version_info.platform_type == com_data.firmware_version.platform_type) &&
        (current_firmware_version_info.application_type == com_data.firmware_version.application_type)
        // ignore Git Hash for now because CPX is not sending it in current firmware implementation
        // (current_git_hash_info.hash_signal == com_data.git_hash.hash_signal)
    ) {
        EVLOG_error << "Could not determine CPX firmware information.";
        // throw std::runtime_error("Could not determine CPX firmware information.");
    }

    this->fw_info = std::string(std::to_string(can_firmware_version_major_version_decode(com_data.firmware_version.major_version))) +
                    std::string(std::to_string(can_firmware_version_minor_version_decode(com_data.firmware_version.minor_version))) +
                    std::string(std::to_string(can_firmware_version_build_version_decode(com_data.firmware_version.build_version))) +
                    " (g" +
                    std::string(std::to_string(can_git_hash_hash_signal_decode(com_data.git_hash.hash_signal))) +
                    ", " +
                    std::string(std::to_string(can_firmware_version_platform_type_decode(com_data.firmware_version.platform_type))) +
                    std::string(std::to_string(can_firmware_version_application_type_decode(com_data.firmware_version.application_type)));

    fv_lock.unlock();
    gh_lock.unlock();

    // signal new value
    if (!this->has_cpx_connected_once) {
        EVLOG_info << "Firmware Version and Git Hash received!";
    }
    this->on_fw_info(this->fw_info);
}

void CbCpx::enable() {
    // start sending of periodic Charge Control frames
    this->tx_cc_enabled = true;

    // start reacting on received CAN BCM messages
    this->rx_bcm_enabled = true;

    // start reacting on received CAN RAW messages
    this->rx_raw_enabled = true;
}

canid_t CbCpx::get_can_id(int cpx_id, int message_id) {
    canid_t base_id;

    if (cpx_id <= 0) {
        base_id = message_id;
    } else {
        // Beispiel: cpx_id im oberen Bereich, message_id in den unteren Bits
        base_id = ((static_cast<canid_t>(cpx_id) & 0x1FFFFF) << 8) |  // obere Bits
                  (static_cast<canid_t>(message_id) & 0xFF);          // untere Bits
    }

    // Sicherstellen, dass wir in den 29-Bit-Bereich passen
    base_id &= CAN_EFF_MASK;

    // Extended-Frame-Flag setzen
    return base_id | CAN_EFF_FLAG;
}

void CbCpx::can_bcm_rx_init() {
    struct {
        struct bcm_msg_head msg_head;
        struct can_frame frame;
    } rx_setup;

    rx_setup.msg_head.opcode  = RX_SETUP;
    rx_setup.msg_head.can_id  = get_can_id(this->config.id, CAN_CHARGE_STATE1_FRAME_ID);
    rx_setup.msg_head.flags   = RX_FILTER_ID | SETTIMER;
    rx_setup.msg_head.nframes = 1;
    
    rx_setup.msg_head.ival1.tv_sec = 0;
    rx_setup.msg_head.ival1.tv_usec = 120000;
    rx_setup.msg_head.ival2.tv_sec = 0;
    rx_setup.msg_head.ival2.tv_usec = 120000;

    rx_setup.frame.can_id = rx_setup.msg_head.can_id;
    rx_setup.frame.can_dlc = CAN_CHARGE_STATE1_LENGTH;
    memset(rx_setup.frame.data, 0x00, CAN_CHARGE_STATE1_LENGTH);
    
    // init Charge State data
    memset(cs_msg.frame.data, 0x00, CAN_CHARGE_STATE1_LENGTH);
    cs_msg.msg_head.can_id = get_can_id(this->config.id, CAN_CHARGE_STATE1_FRAME_ID);

    if (write(this->can_bcm_rx_fd, &rx_setup, sizeof(rx_setup)) < 0) {
        throw std::system_error(errno, std::generic_category(), "Charge State RX setup failed!");
    }

    rx_setup.msg_head.can_id  = get_can_id(this->config.id, CAN_PT1000_STATE_FRAME_ID);
    rx_setup.frame.can_dlc = CAN_PT1000_STATE_LENGTH;
    memset(rx_setup.frame.data, 0x00, CAN_PT1000_STATE_LENGTH);
    
    // init PT1000 State data
    memset(pt_msg.frame.data, 0x00, CAN_PT1000_STATE_LENGTH);
    pt_msg.msg_head.can_id = get_can_id(this->config.id, CAN_PT1000_STATE_FRAME_ID);

    if (write(this->can_bcm_rx_fd, &rx_setup, sizeof(rx_setup)) < 0) {
        throw std::system_error(errno, std::generic_category(), "PT1000 State RX setup failed!");
    }
}

void CbCpx::charge_control_update() {
    struct {
        struct bcm_msg_head msg_head;
        struct can_frame frame;
    } msg_delete;

    struct {
        struct bcm_msg_head msg_head;
        struct can_frame frame;
    } msg_setup;
    
    if (this->tx_cc_enabled) {
        uint8_t payload[8];

        // only delete previous message if it was initialized before
        if (this->charge_control_initialized) {
            memset(&msg_delete, 0, sizeof(msg_delete));
            msg_delete.msg_head.opcode  = TX_DELETE;
            msg_delete.msg_head.can_id  = get_can_id(this->config.id, CAN_CHARGE_CONTROL1_FRAME_ID);
            msg_delete.msg_head.nframes = 0;

            if (write(this->can_bcm_tx_fd, &msg_delete, sizeof(msg_delete)) < 0) {
                throw std::system_error(errno, std::generic_category(), "Charge Control delete failed!");
            }
        }

        msg_setup.msg_head.opcode  = TX_SETUP;
        msg_setup.msg_head.can_id  = get_can_id(this->config.id, CAN_CHARGE_CONTROL1_FRAME_ID);
        msg_setup.msg_head.flags   = SETTIMER | STARTTIMER;
        msg_setup.msg_head.nframes = 1;
        msg_setup.msg_head.count   = 0;

        msg_setup.msg_head.ival1.tv_sec = 0;
        msg_setup.msg_head.ival1.tv_usec = 0;
        msg_setup.msg_head.ival2.tv_sec = 0;
        msg_setup.msg_head.ival2.tv_usec = 100000;

        std::unique_lock<std::mutex> cc_lock(this->cc_mutex);

        can_charge_control1_pack(payload, &com_data.charge_control, CAN_CHARGE_CONTROL1_LENGTH);
        
        msg_setup.frame.can_id = get_can_id(this->config.id, CAN_CHARGE_CONTROL1_FRAME_ID);
        msg_setup.frame.len = CAN_CHARGE_CONTROL1_LENGTH;
        memcpy(msg_setup.frame.data, payload, CAN_CHARGE_CONTROL1_LENGTH);

        cc_lock.unlock();
        
        if (write(this->can_bcm_tx_fd, &msg_setup, sizeof(msg_setup)) < 0) {
            throw std::system_error(errno, std::generic_category(), "Charge Control init failed!");
        }

        this->charge_control_initialized = true;

    } else {
        throw std::system_error(errno, std::generic_category(), "Enable sending of Charge Control before updating it!");
    }
}

void CbCpx::set_duty_cycle(unsigned int duty_cycle) {
    std::unique_lock<std::mutex> cc_lock(this->cc_mutex);

    com_data.charge_control.cc_target_duty_cycle = duty_cycle;
    com_data.charge_control.cc_pwm_active = 1;

    cc_lock.unlock();

    this->charge_control_update();
    
    // only check if CPX is not timed out
    // warning about CPX timeout is provided on noticing it
    if (!this->bcm_rx_timeout && this->has_cpx_connected_once) {
        this->launch_duty_cycle_check(duty_cycle);
    }
}

unsigned int CbCpx::get_duty_cycle() {
    return get_cs_current_duty_cycle();
}

int CbCpx::switch_state(bool on) {
    std::unique_lock<std::mutex> cc_lock(this->cc_mutex);

    com_data.charge_control.cc_contactor1_state = on;
    com_data.charge_control.cc_contactor2_state = on;

    cc_lock.unlock();

    this->charge_control_update();

    // if CPX timedout ignore that contactors are not switching
    if (this->bcm_rx_timeout) {
        return 0;
    }

    // we should see the changes take effect after 1s (FIXME)
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (get_cs_contactor_state(1) != on) {
        return 1;
    } else if ((get_cs_contactor_state(2) != on)) {
        return 2;
    }

    return 0;
}

bool CbCpx::get_contactor_state_no_lock() {
    unsigned int i;
    bool at_least_one_is_configured = false;
    bool target_state = false;
    bool actual_state = false;

    for (i = 1; i <= CB_PROTO_MAX_CONTACTORS; ++i) {
        if (get_cs_contactor_state(i) == CAN_CHARGE_STATE1_CS_CONTACTOR1_STATE_OPEN_CHOICE ||
            get_cs_contactor_state(i) == CAN_CHARGE_STATE1_CS_CONTACTOR1_STATE_CLOSE_CHOICE) {
            at_least_one_is_configured = true;

            // don't overwrite, but merge the state
            actual_state |= bool(get_cs_contactor_state(i));
        }

        // fallback in the same loop in case no contactor is actually in use
        // don't overwrite, but merge the state
        target_state |= bool(get_cc_contactor_state(i));
    }

    if (at_least_one_is_configured)
        return actual_state;
    else
        return target_state;
}

bool CbCpx::get_contactor_state() {
    return this->get_contactor_state_no_lock();
}

unsigned int CbCpx::get_temperature_channels() const {
    return CB_PROTO_MAX_PT1000S;
}

bool CbCpx::is_temperature_enabled(unsigned int channel) {
    return get_pt1000_is_active(channel);
}

types::board_support_common::Ampacity CbCpx::pp_state_to_ampacity(uint8_t pp_state) {
    // we map only the well-known states in this method - for all other a std::runtime_error is raised    
    switch (pp_state) {
    case CAN_CHARGE_STATE1_CS_CURRENT_PP_STATE_NO_CABLE_DETECTED_CHOICE:
        return types::board_support_common::Ampacity::None;

    case CAN_CHARGE_STATE1_CS_CURRENT_PP_STATE_13_A_CHOICE:
        return types::board_support_common::Ampacity::A_13;

    case CAN_CHARGE_STATE1_CS_CURRENT_PP_STATE_20_A_CHOICE:
        return types::board_support_common::Ampacity::A_20;

    case CAN_CHARGE_STATE1_CS_CURRENT_PP_STATE_32_A_CHOICE:
        return types::board_support_common::Ampacity::A_32;

    case CAN_CHARGE_STATE1_CS_CURRENT_PP_STATE_63_70_A_CHOICE:
        return types::board_support_common::Ampacity::A_63_3ph_70_1ph;

    default:
        EVLOG_error << "The measured voltage for the Proximity Pilot could not be mapped.";
        return types::board_support_common::Ampacity::None;
    }
}

types::board_support_common::Ampacity CbCpx::get_ampacity() {
    std::scoped_lock cs_lock(this->cs_mutex);
    EVLOG_info << "pp_state - " << this->pp_state_to_ampacity(get_cs_current_pp_state());
    return this->pp_state_to_ampacity(get_cs_current_pp_state());
}

uint16_t CbCpx::get_cs_current_duty_cycle() {
    std::unique_lock<std::mutex> cs_lock(this->cs_mutex);
    return com_data.charge_state.cs_current_duty_cycle;
}

uint8_t CbCpx::get_cs_pwm_active() {
    std::unique_lock<std::mutex> cs_lock(this->cs_mutex);
    return com_data.charge_state.cs_pwm_active;
}

uint8_t CbCpx::get_cs_current_cp_state() {
    std::unique_lock<std::mutex> cs_lock(this->cs_mutex);
    return com_data.charge_state.cs_current_cp_state;
}

uint8_t CbCpx::get_cs_short_circuit() {
    std::unique_lock<std::mutex> cs_lock(this->cs_mutex);
    return com_data.charge_state.cs_cp_short_circuit;
}

uint8_t CbCpx::get_cs_diode_fault() {
    std::unique_lock<std::mutex> cs_lock(this->cs_mutex);
    return com_data.charge_state.cs_diode_fault;
}

uint8_t CbCpx::get_cs_current_pp_state() {
    std::unique_lock<std::mutex> cs_lock(this->cs_mutex);
    return com_data.charge_state.cs_current_pp_state;
}

uint8_t CbCpx::get_cs_contactor_state(int contactor) {
    std::unique_lock<std::mutex> cs_lock(this->cs_mutex);
    if (contactor == 1) {
        return com_data.charge_state.cs_contactor1_state;
    } else if (contactor == 2) {
        return com_data.charge_state.cs_contactor2_state;
    } else {
        throw std::system_error(errno, std::generic_category(), "Selected contactor out of range!");
    }
}

uint8_t CbCpx::get_cc_contactor_state(int contactor) {
    std::unique_lock<std::mutex> cc_lock(this->cc_mutex);
    if (contactor == 1) {
        return com_data.charge_control.cc_contactor1_state;
    } else if (contactor == 2) {
        return com_data.charge_control.cc_contactor2_state;
    } else {
        throw std::system_error(errno, std::generic_category(), "Selected contactor out of range!");
    }
}

bool CbCpx::is_cs_contactor_error(int contactor = 0) {
    std::unique_lock<std::mutex> cs_lock(this->cs_mutex);
    if (contactor == 1) {
        return com_data.charge_state.cs_contactor1_error;
    } else if (contactor == 2) {
        return com_data.charge_state.cs_contactor2_error;
    } else if (contactor == 0) {
        return (com_data.charge_state.cs_contactor1_error || com_data.charge_state.cs_contactor2_error);
    } else {
        throw std::system_error(errno, std::generic_category(), "Selected contactor out of range!");
    }
}

uint8_t CbCpx::get_cs_hv_ready() {
    std::unique_lock<std::mutex> cs_lock(this->cs_mutex);
    return com_data.charge_state.cs_hv_ready;
}

bool CbCpx::is_cs_estop_charging_abort(int estop = 0) {
    std::unique_lock<std::mutex> cs_lock(this->cs_mutex);
    if (estop == 1) {
        if (com_data.charge_state.cs_estop1_charging_abort == 1) {
            return true;
        } else {
            return false;
        }
    } else if (estop == 2) {
        if (com_data.charge_state.cs_estop2_charging_abort == 1) {
            return true;
        } else {
            return false;
        }
    } else if (estop == 3) {
        if (com_data.charge_state.cs_estop3_charging_abort == 1) {
            return true;
        } else {
            return false;
        }
    } else if (estop == 0) {
        if ((com_data.charge_state.cs_estop1_charging_abort == 1) or
            (com_data.charge_state.cs_estop2_charging_abort == 1) or
            (com_data.charge_state.cs_estop3_charging_abort == 1)) {
                return true;
        } else {
            return false;
        }
    } else {
        throw std::system_error(errno, std::generic_category(), "Selected estop out of range!");
    }
}

bool CbCpx::get_pt1000_is_active(int channel) {
    std::unique_lock<std::mutex> pt_lock(this->pt_mutex);
    if (channel == 1) {
        return com_data.pt1000_state.pt1_temperature != CAN_PT1000_STATE_PT1_TEMPERATURE_TEMP_SENSOR_NOT_USED_CHOICE;
    } else if (channel == 2) {
        return com_data.pt1000_state.pt2_temperature != CAN_PT1000_STATE_PT2_TEMPERATURE_TEMP_SENSOR_NOT_USED_CHOICE;
    } else if (channel == 3) {
        return com_data.pt1000_state.pt3_temperature != CAN_PT1000_STATE_PT3_TEMPERATURE_TEMP_SENSOR_NOT_USED_CHOICE;
    } else if (channel == 4) {
        return com_data.pt1000_state.pt4_temperature != CAN_PT1000_STATE_PT4_TEMPERATURE_TEMP_SENSOR_NOT_USED_CHOICE;
    } else {
        throw std::system_error(errno, std::generic_category(), "Selected PT1000 channel out of range!");
    }
}

bool CbCpx::is_temperature_valid(unsigned int channel) {
    std::unique_lock<std::mutex> pt_lock(this->pt_mutex);

    if (channel == 1) {
        return !(com_data.pt1000_state.pt1_selftest_failed && com_data.pt1000_state.pt1_charging_stopped);
    } else if (channel == 2) {
        return !(com_data.pt1000_state.pt2_selftest_failed && com_data.pt1000_state.pt2_charging_stopped);
    } else if (channel == 3) {
        return !(com_data.pt1000_state.pt3_selftest_failed && com_data.pt1000_state.pt3_charging_stopped);
    } else if (channel == 4) {
        return !(com_data.pt1000_state.pt4_selftest_failed && com_data.pt1000_state.pt4_charging_stopped);
    } else {
        throw std::system_error(errno, std::generic_category(), "Selected PT1000 channel out of range!");
    }
}

bool CbCpx::is_pt_selftest_failed(unsigned int channel = 0) {
    std::unique_lock<std::mutex> pt_lock(this->pt_mutex);

    if (channel == 1) {
        return com_data.pt1000_state.pt1_selftest_failed;
    } else if (channel == 2) {
        return com_data.pt1000_state.pt2_selftest_failed;
    } else if (channel == 3) {
        return com_data.pt1000_state.pt3_selftest_failed;
    } else if (channel == 4) {
        return com_data.pt1000_state.pt4_selftest_failed;
    } else if (channel == 0) {
        return (com_data.pt1000_state.pt1_selftest_failed ||
                com_data.pt1000_state.pt2_selftest_failed ||
                com_data.pt1000_state.pt3_selftest_failed ||
                com_data.pt1000_state.pt4_selftest_failed);
    } else {
        throw std::system_error(errno, std::generic_category(), "Selected PT1000 channel out of range!");
    }
}

bool CbCpx::is_pt_charging_stopped(unsigned int channel = 0) {
    std::unique_lock<std::mutex> pt_lock(this->pt_mutex);

    if (channel == 1) {
        return com_data.pt1000_state.pt1_charging_stopped;
    } else if (channel == 2) {
        return com_data.pt1000_state.pt2_charging_stopped;
    } else if (channel == 3) {
        return com_data.pt1000_state.pt3_charging_stopped;
    } else if (channel == 4) {
        return com_data.pt1000_state.pt4_charging_stopped;
    } else if (channel == 0) {
        return (com_data.pt1000_state.pt1_charging_stopped ||
                com_data.pt1000_state.pt2_charging_stopped ||
                com_data.pt1000_state.pt3_charging_stopped ||
                com_data.pt1000_state.pt4_charging_stopped);
    }
    else {
        throw std::system_error(errno, std::generic_category(), "Selected PT1000 channel out of range!");
    }
}

float CbCpx::get_temperature(unsigned int channel) {
    std::unique_lock<std::mutex> pt_lock(this->pt_mutex);

    if (channel == 1) {
        return com_data.pt1000_state.pt1_temperature;
    } else if (channel == 2) {
        return com_data.pt1000_state.pt2_temperature;
    } else if (channel == 3) {
        return com_data.pt1000_state.pt3_temperature;
    } else if (channel == 4) {
        return com_data.pt1000_state.pt4_temperature;
    } else {
        throw std::system_error(errno, std::generic_category(), "Selected PT1000 channel out of range!");
    }
}

bool CbCpx::is_emergency() {
    bool pp_error = false;

    if (this->is_pluggable) {
        // any state above this is considered an error for now
        pp_error = get_cs_current_pp_state() > CAN_CHARGE_STATE1_CS_CURRENT_PP_STATE_63_70_A_CHOICE;
    }

    return pp_error or (get_cs_current_cp_state() == CAN_CHARGE_STATE1_CS_CURRENT_CP_STATE_INVALID_CHOICE) or
           (get_cs_short_circuit() != 0) or (get_cs_diode_fault() != 0) or is_cs_contactor_error() or
           is_cs_estop_charging_abort() or is_pt_selftest_failed() or is_pt_charging_stopped();
}

void CbCpx::read_charge_state(uint8_t* data) {
    std::unique_lock<std::mutex> cs_lock(this->cs_mutex);

    can_charge_state1_unpack(&com_data.charge_state, data, CAN_CHARGE_STATE1_LENGTH);

    cs_lock.unlock();
}

void CbCpx::read_pt1000_state(uint8_t* data) {
    std::unique_lock<std::mutex> pt_lock(this->pt_mutex);

    can_pt1000_state_unpack(&com_data.pt1000_state, data, CAN_PT1000_STATE_LENGTH);

    // multiply temperatures with correct scale
    com_data.pt1000_state.pt1_temperature *=  CB_PROTO_TEMP_SCALE;
    com_data.pt1000_state.pt2_temperature *=  CB_PROTO_TEMP_SCALE;
    com_data.pt1000_state.pt3_temperature *=  CB_PROTO_TEMP_SCALE;
    com_data.pt1000_state.pt4_temperature *=  CB_PROTO_TEMP_SCALE;

    pt_lock.unlock();
}

void CbCpx::read_fw_version(uint8_t* data) {
    std::unique_lock<std::mutex> fv_lock(this->fv_mutex);
    can_firmware_version_unpack(&com_data.firmware_version, data, CAN_FIRMWARE_VERSION_LENGTH);
}

void CbCpx::read_git_hash(uint8_t* data) {
    std::unique_lock<std::mutex> gh_lock(this->gh_mutex);
    can_git_hash_unpack(&com_data.git_hash, data, CAN_GIT_HASH_LENGTH);
}

bool CbCpx::is_any_notify_flag_set() {
    return notify_flags.pp_changed ||
           notify_flags.cp_changed ||
           notify_flags.cp_error ||
           notify_flags.contactor_1_error ||
           notify_flags.contactor_2_error ||
           notify_flags.estop_1_changed ||
           notify_flags.estop_2_changed ||
           notify_flags.estop_3_changed;
}

bool CbCpx::is_contactor_error(int contactor) {
    if (((get_cs_contactor_state(contactor) == CAN_CHARGE_STATE1_CS_CONTACTOR1_STATE_OPEN_CHOICE) or
        (get_cs_contactor_state(contactor) == CAN_CHARGE_STATE1_CS_CONTACTOR1_STATE_CLOSE_CHOICE)) and
        is_cs_contactor_error(contactor)) {
        return true;
    } else {
        return false;
    }
}

void CbCpx::launch_duty_cycle_check(unsigned int expected_duty_cycle) {
    // stop previous thread if running
    if (this->duty_cycle_check_thread.joinable()) {
        this->duty_cycle_check_termination_requested = true;
        this->duty_cycle_check_cv.notify_all();
        this->duty_cycle_check_thread.join();
    }

    this->duty_cycle_check_termination_requested = false;

    this->duty_cycle_check_thread = std::thread([this, expected_duty_cycle]() {
        std::unique_lock<std::mutex> lock(this->duty_cycle_check_mutex);
        while(!this->duty_cycle_check_termination_requested) {
            bool notified = this->duty_cycle_check_cv.wait_for(
                lock, std::chrono::seconds(1),
                [this]{ return this->duty_cycle_check_termination_requested.load(); }
            );

            if (this->duty_cycle_check_termination_requested || notified)
                break;

            double duty_cycle = expected_duty_cycle / 10.0;
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(1) << duty_cycle;

            if (expected_duty_cycle == get_cs_current_duty_cycle()) {
                EVLOG_debug << "[suppressed] Safety Controller did accept the new duty cycle of " << oss.str() << "%";
            } else {
                EVLOG_warning << "Safety Controller did not accept the new duty cycle of " << oss.str() << "%";
            }

            break;
        }
    });
}

void CbCpx::handle_timeout_watchdog(int flag) {
    // disable thread in case CPX reports back
    if (flag == 0) {
        this->timeout_watchdog_termination_requested = true;
        this->timeout_watchdog_cv.notify_all();
        if (this->timeout_watchdog_thread.joinable()) {
            this->timeout_watchdog_thread.join();
        }
        this->timeout_watchdog_termination_requested = false;
        this->bcm_rx_timeout = false;
        EVLOG_info << "Receiving on BCM socket after timeout - CPX-ID: " << std::to_string(this->config.id);
        this->on_cpx_timeout(false);
        return;
    }

    if (this->timeout_watchdog_thread.joinable()) {
        timeout_watchdog_termination_requested = true;
        timeout_watchdog_cv.notify_all();
        this->timeout_watchdog_thread.join();
    }

    this->timeout_watchdog_termination_requested = false;

    this->timeout_watchdog_thread = std::thread([this] {
        std::unique_lock<std::mutex> lock(this->timeout_watchdog_mutex);
        while (!this->timeout_watchdog_termination_requested) {
            bool notified = this->timeout_watchdog_cv.wait_for(
                lock, std::chrono::seconds(this->cpx_timeout_seconds),
                [this]{ return this->timeout_watchdog_termination_requested.load(); });

            if (this->timeout_watchdog_termination_requested || notified)
                break;

            if (this->bcm_rx_timeout) {
                EVLOG_warning << "Failed to receive on BCM socket - CPX-ID: " << std::to_string(this->config.id);
                this->on_cpx_timeout(true);
                break;
            }
        }
    });
}

void CbCpx::notify_worker() {
    uint8_t previous_cp_state = CAN_CHARGE_STATE1_CS_CURRENT_CP_STATE_INVALID_CHOICE + 1;
    uint8_t previous_pp_state = CAN_CHARGE_STATE1_CS_CURRENT_PP_STATE_ERROR_CHOICE + 1;

    uint8_t current_cp_state;
    uint8_t current_pp_state;

    while (!this->termination_requested) {
        if (this->termination_requested)
            break;

        // react to changes relevant for notification
        std::unique_lock<std::mutex> lock(this->notify_mutex);

        notify_worker_cv.wait(lock, [&]{ 
            return termination_requested || is_any_notify_flag_set();
        });

        bool pp_changed = std::exchange(notify_flags.pp_changed, false);
        bool cp_changed = std::exchange(notify_flags.cp_changed, false);
        bool estop_1_changed = std::exchange(notify_flags.estop_1_changed, false);
        bool estop_2_changed = std::exchange(notify_flags.estop_2_changed, false);
        bool estop_3_changed = std::exchange(notify_flags.estop_3_changed, false);
        bool cp_error = std::exchange(notify_flags.cp_error, false);
        bool contactor_1_error = std::exchange(notify_flags.contactor_1_error, false);
        bool contactor_2_error = std::exchange(notify_flags.contactor_2_error, false);
        lock.unlock();

        // check for PP changes
        if (pp_changed) {
            current_pp_state = get_cs_current_pp_state();
            if (previous_pp_state != (CAN_CHARGE_STATE1_CS_CURRENT_PP_STATE_ERROR_CHOICE + 1)) {
                if (this->is_pluggable) {
                    EVLOG_debug << "on_pp_change(" << pp_state_to_ampacity(current_pp_state) << ")";
                    this->on_pp_change(current_pp_state);
                } else {
                    EVLOG_debug << "on_pp_change(" << pp_state_to_ampacity(current_pp_state)
                               << ") [suppressed, fixed cable]";
                }
            } else {
                EVLOG_debug << "on_pp_change(" << pp_state_to_ampacity(current_pp_state) << ") [suppressed]";
            }
            previous_pp_state = current_pp_state;
        }

        // check for CP changes
        if (cp_changed) {
            current_cp_state = get_cs_current_cp_state();
            if (previous_cp_state != CAN_CHARGE_STATE1_CS_CURRENT_CP_STATE_UNKNOWN_CHOICE) {
                EVLOG_debug << "on_cp_change(" << static_cast<types::cb_board_support::CPState>(current_cp_state) << ")";
                this->on_cp_change(current_cp_state);
            } else {
                EVLOG_debug << "on_cp_change(" << static_cast<types::cb_board_support::CPState>(current_cp_state) << ") [suppressed]";
            }
            previous_cp_state = current_cp_state;
        }

        // check for ESTOPs
        if (estop_1_changed) {
            this->on_estop(1, is_cs_estop_charging_abort(1));
        }

        if (estop_2_changed) {
            this->on_estop(2, is_cs_estop_charging_abort(2));
        }

        if (estop_3_changed) {
            this->on_estop(3, is_cs_estop_charging_abort(3));
        }

        // check for contactor errors
        if (contactor_1_error) {
            std::string name = "Contactor 1";
            this->on_contactor_error(name, get_cc_contactor_state(1),
                                     get_cs_contactor_state(1)
                                         ? types::cb_board_support::ContactorState::Closed
                                         : types::cb_board_support::ContactorState::Open);
        }

        if (contactor_2_error) {
            std::string name = "Contactor 2";
            this->on_contactor_error(name, get_cc_contactor_state(2),
                                     get_cs_contactor_state(2)
                                         ? types::cb_board_support::ContactorState::Closed
                                         : types::cb_board_support::ContactorState::Open);
        }

        // check for CP related errors
        if (cp_error) {
            this->on_cp_error();
        }
    }

    EVLOG_info << "Notify Thread terminated";
}

void CbCpx::can_bcm_rx_worker() {
    struct {
        struct bcm_msg_head msg_head;
        struct can_frame frame;
    } msg;

    unsigned int i;

    // Snapshot of CAN-derived states used to decide which notify flags must fire.
    struct NotifyState {
        uint8_t pp_state {0};
        uint8_t cp_state {0};
        bool estop[CB_PROTO_MAX_ESTOPS] {false, false, false};
        unsigned int cp_errors {0};
        bool contactor_errors[CB_PROTO_MAX_CONTACTORS] {false, false};  
    };

    // Capture the current notification state so we can compare before/after decoding a frame.
    auto capture_notify_state = [this]() {
        NotifyState state;
        state.pp_state = get_cs_current_pp_state();
        state.cp_state = get_cs_current_cp_state();
        for (unsigned int i = 0; i < CB_PROTO_MAX_ESTOPS; ++i) {
            state.estop[i] = is_cs_estop_charging_abort(i + 1);
        }
        state.cp_errors = (static_cast<unsigned int>(get_cs_diode_fault()) << 1) | static_cast<unsigned int>(get_cs_short_circuit());
        for (unsigned int i = 0; i < CB_PROTO_MAX_CONTACTORS; ++i) {
            state.contactor_errors[i] = is_contactor_error(i + 1);
        }
        return state;
    };

    EVLOG_info << "CAN BCM Rx Thread started";

    while (!this->termination_requested) {
        if (rx_bcm_enabled) {
            ssize_t rv = read(this->can_bcm_rx_fd, &msg, sizeof(msg));

            if (rv < 0) {
                EVLOG_warning << "Couldn't read event from BCM";
            }
            if (rv == 0)
                continue; // this should usually not happen
            // we should receive either a header plus frame, or only the header in case of RX_TIMEOUT
            if (rv < static_cast<ssize_t>(sizeof(bcm_msg_head) + msg.msg_head.nframes * sizeof(can_frame))) {
                EVLOG_warning << "Short CAN BCM read";
            }

            // check for timeout or received data
            if (msg.msg_head.opcode == RX_TIMEOUT) {
                this->bcm_rx_timeout = true;
                this->handle_timeout_watchdog(1);

            } else if (msg.msg_head.opcode == RX_CHANGED) {
                // compare current and received data by CAN-ID
                if ((msg.msg_head.can_id == this->cs_msg.msg_head.can_id) && (memcmp(msg.frame.data, this->cs_msg.frame.data, 8) != 0)) {                    
                    // remember new frame as current
                    cs_msg.msg_head = msg.msg_head;
                    cs_msg.frame = msg.frame;

                    // remember current notify state
                    const auto previous_notify_state = capture_notify_state();

                    this->read_charge_state(msg.frame.data);

                    // remember new notify state
                    const auto new_notify_state = capture_notify_state();

                    bool notify_required = false;

                    auto mark_change = [&](bool changed, bool& flag) {
                        if (changed) {
                            flag = true;
                            notify_required = true;
                        }
                    };

                    {
                        std::lock_guard<std::mutex> lock(this->notify_mutex);

                        mark_change(previous_notify_state.pp_state != new_notify_state.pp_state, notify_flags.pp_changed);
                        mark_change(previous_notify_state.cp_state != new_notify_state.cp_state, notify_flags.cp_changed);
                        mark_change(previous_notify_state.cp_errors != new_notify_state.cp_errors, notify_flags.cp_error);
                        mark_change(previous_notify_state.contactor_errors[0] != new_notify_state.contactor_errors[0], notify_flags.contactor_1_error);
                        mark_change(previous_notify_state.contactor_errors[1] != new_notify_state.contactor_errors[1], notify_flags.contactor_2_error);
                        mark_change(previous_notify_state.estop[0] != new_notify_state.estop[0], notify_flags.estop_1_changed);
                        mark_change(previous_notify_state.estop[1] != new_notify_state.estop[1], notify_flags.estop_2_changed);
                        mark_change(previous_notify_state.estop[2] != new_notify_state.estop[2], notify_flags.estop_3_changed);
                    }

                    if (notify_required) {
                        notify_worker_cv.notify_one();
                    }

                } else if ((msg.msg_head.can_id == this->pt_msg.msg_head.can_id) && (memcmp(msg.frame.data, this->pt_msg.frame.data, 8) != 0)) {
                    // remember new frame as current
                    pt_msg.msg_head = msg.msg_head;
                    pt_msg.frame = msg.frame;

                    // make new received data available
                    this->read_pt1000_state(msg.frame.data);
                    this->temperature_data_is_valid = true;

                } else if ((msg.msg_head.can_id != this->cs_msg.msg_head.can_id) && (msg.msg_head.can_id != this->pt_msg.msg_head.can_id)) {
                    EVLOG_info << "[RECV] CAN ID: 0x" << std::hex << msg.msg_head.can_id;
                    EVLOG_info << "[Charge State] CAN ID: 0x" << std::hex << this->cs_msg.msg_head.can_id;
                    EVLOG_info << "[PT1000 State] CAN ID: 0x" << std::hex << this->pt_msg.msg_head.can_id;
                    EVLOG_warning << "CAN BCM RX: Unknown CAN-ID received";
                }

                if ((msg.msg_head.can_id == this->cs_msg.msg_head.can_id) or (msg.msg_head.can_id == this->pt_msg.msg_head.can_id)) {
                    // do stuff in case CPX was not yet connected
                    if (!this->has_cpx_connected_once) {
                        // request firmware version and git hash on first connection
                        get_firmware_and_git_hash();

                        // check if CPX accepted duty cycle
                        this->launch_duty_cycle_check(com_data.charge_control.cc_target_duty_cycle);
                        // // wait in seperate thread
                        // // we should see the changes take effect after 1s (FIXME)
                        // std::thread wait_thread([&]() {
                        //     std::this_thread::sleep_for(std::chrono::seconds(1));
                        //     double duty_cycle = com_data.charge_control.cc_target_duty_cycle / 10.0;
                        //     std::ostringstream oss;
                        //     oss << std::fixed << std::setprecision(1) << duty_cycle;

                        //     if (com_data.charge_control.cc_target_duty_cycle == get_cs_current_duty_cycle()) {
                        //         EVLOG_info << "Safety Controller did accept the new duty cycle of " << oss.str() << "%";
                        //     } else {
                        //         EVLOG_warning << "Safety Controller did not accept the new duty cycle of " << oss.str() << "%";
                        //     }
                        // });
                        // wait_thread.detach();

                        this->has_cpx_connected_once = true;
                    }

                    // if expected CAN-ID was received and timeout active
                    // reset timeout flag and report back
                    if (this->bcm_rx_timeout == true) {
                        this->handle_timeout_watchdog(0);
                    }
                }
            } else {
                EVLOG_warning << "Unexpected BCM opcode received: " << std::to_string(msg.msg_head.opcode);
            }
        }
    }

    EVLOG_info << "CAN BCM Rx Thread stopped";
}

void CbCpx::can_raw_rx_worker() {
    struct can_frame frame;

    EVLOG_info << "CAN RAW Rx Thread started";

    while (!this->termination_requested) {        
        if (rx_raw_enabled) {
            int rv;

            // read blocks until new frame is available
            rv = read(this->can_raw_fd, &frame, sizeof(frame));
            if (rv < 0) {
                EVLOG_warning << "Couldn't read event from RAW";
                // throw std::system_error(errno, std::generic_category(), "Couldn't read event from RAW");
            }
            if (rv == 0 || rv < (int)sizeof(struct can_frame))
                continue; // this should usually not happen

            if ((frame.can_id) == get_can_id(this->config.id, CAN_FIRMWARE_VERSION_FRAME_ID)) {
                this->read_fw_version(frame.data);
            }

            if ((frame.can_id) == get_can_id(this->config.id, CAN_GIT_HASH_FRAME_ID)) {
                this->read_git_hash(frame.data);
            }
        }
    }

    EVLOG_info << "CAN RAW Rx Thread stopped";
}
