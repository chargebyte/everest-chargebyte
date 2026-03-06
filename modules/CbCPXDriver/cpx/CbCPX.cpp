// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include "CbCPX.hpp"
#include <libsocketcan.h>

using namespace std::chrono_literals;

CbCPX::CbCPX(int device_id, const std::string& can_interface, int can_bitrate) {
    // init EVerest-config parameters
    this->config.device_id = device_id;
    this->config.can_interface = can_interface;
    this->config.can_bitrate = can_bitrate;

    // we have to convert can_interface of type std::string to const char *
    const char* can_interface_cstr = this->config.can_interface.c_str();

    // open a CAN BCM RX socket, bound to the desired interface
    this->can_bcm_rx_fd = socket(PF_CAN, SOCK_DGRAM, CAN_BCM);
    if (this->can_bcm_rx_fd == -1)
        throw std::system_error(errno, std::generic_category(), "socket(PF_CAN, CAN_BCM) failed");

    strncpy(this->ifr.ifr_name, can_interface_cstr, sizeof(this->ifr.ifr_name));
    if (ioctl(this->can_bcm_rx_fd, SIOCGIFINDEX, &this->ifr))
        throw std::system_error(errno, std::generic_category(),
                                "Couldn't determine interface number of " + this->config.can_interface);

    this->addr.can_family = AF_CAN;
    this->addr.can_ifindex = this->ifr.ifr_ifindex;

    if (connect(this->can_bcm_rx_fd, reinterpret_cast<sockaddr*>(&this->addr), sizeof(this->addr)))
        throw std::system_error(errno, std::generic_category(),
                                "Couldn't connect Rx CAN BCM socket on " + this->config.can_interface);

    // open a CAN BCM RX socket, bound to the desired interface
    this->can_bcm_tx_fd = socket(PF_CAN, SOCK_DGRAM, CAN_BCM);
    if (this->can_bcm_tx_fd == -1)
        throw std::system_error(errno, std::generic_category(), "socket(PF_CAN, CAN_BCM) failed");

    strncpy(this->ifr.ifr_name, can_interface_cstr, sizeof(this->ifr.ifr_name));
    if (ioctl(this->can_bcm_tx_fd, SIOCGIFINDEX, &this->ifr))
        throw std::system_error(errno, std::generic_category(),
                                "Couldn't determine interface number of " + this->config.can_interface);

    if (connect(this->can_bcm_tx_fd, reinterpret_cast<sockaddr*>(&this->addr), sizeof(this->addr)))
        throw std::system_error(errno, std::generic_category(),
                                "Couldn't connect Tx CAN BCM socket on " + this->config.can_interface);

    // open a CAN RAW socket, bound to the desired interface
    this->can_raw_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (this->can_raw_fd == -1)
        throw std::system_error(errno, std::generic_category(), "socket(PF_CAN, CAN_RAW) failed");

    // set interface name
    std::strncpy(this->ifr.ifr_name, can_interface_cstr, IFNAMSIZ - 1);
    if (ioctl(this->can_raw_fd, SIOCGIFINDEX, &this->ifr) < 0) {
        throw std::runtime_error("Failed to get interface index on " + this->config.can_interface);
    }

    // set CAN filters for RAW socket
    struct can_filter filters[2];
    filters[0].can_id = this->get_can_id(this->config.device_id, CAN_FIRMWARE_VERSION_FRAME_ID);
    filters[0].can_mask = CAN_EFF_FLAG | CAN_EFF_MASK;
    filters[1].can_id = this->get_can_id(this->config.device_id, CAN_GIT_HASH_FRAME_ID);
    filters[1].can_mask = CAN_EFF_FLAG | CAN_EFF_MASK;

    if (setsockopt(this->can_raw_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filters, sizeof(filters)) < 0) {
        throw std::system_error(errno, std::generic_category(), "setsockopt(CAN_RAW_FILTER) failed");
    }

    // bind socket
    if (bind(this->can_raw_fd, reinterpret_cast<sockaddr*>(&this->addr), sizeof(this->addr)))
        throw std::system_error(errno, std::generic_category(), "Couldn't bind CAN RAW socket!");

    // setup BCM rx messages
    this->can_bcm_rx_init();

    // launch notify thread
    this->notify_thread = std::thread(&CbCPX::notify_worker, this);

    // launch CAN BCM rx thread
    this->can_bcm_rx_thread = std::thread(&CbCPX::can_bcm_rx_worker, this);

    // launch CAN RAW rx thread
    this->can_raw_rx_thread = std::thread(&CbCPX::can_raw_rx_worker, this);
}

CbCPX::~CbCPX() {
    this->terminate();
}

void CbCPX::init(bool is_pluggable) {
    // remember this setting
    this->is_pluggable = is_pluggable;
}

void CbCPX::terminate() {
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

void CbCPX::ensure_can_bitrate() {
    struct can_bittiming bt {};
    if (can_get_bittiming(this->config.can_interface.c_str(), &bt) != 0) {
        throw std::runtime_error("can_get_bittiming failed: " + std::string(std::strerror(errno)));
    }

    if (static_cast<int>(bt.bitrate) == this->config.can_bitrate) {
        return; // already configured
    }

    if (can_do_stop(this->config.can_interface.c_str()) != 0) {
        throw std::runtime_error("can_do_stop failed: " + std::string(std::strerror(errno)));
    }
    if (can_set_bitrate(this->config.can_interface.c_str(), this->config.can_bitrate) != 0) {
        throw std::runtime_error("can_set_bitrate failed: " + std::string(std::strerror(errno)));
    }
    if (can_do_start(this->config.can_interface.c_str()) != 0) {
        throw std::runtime_error("can_do_start failed: " + std::string(std::strerror(errno)));
    }
}

void CbCPX::get_firmware_and_git_hash() {
    // save current firmware information
    std::unique_lock<std::mutex> fv_lock(this->fv_mutex);
    std::unique_lock<std::mutex> gh_lock(this->gh_mutex);

    const struct can_firmware_version_t current_firmware_version_info = this->com_data.firmware_version;
    // git hash info is not yet being used
    // const struct can_git_hash_t current_git_hash_info = this->com_data.git_hash;

    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    uint8_t payload[8];

    this->print_can_id_info = true;

    // frame information valid for firmware version and git hash
    frame.can_id = this->get_can_id(this->config.device_id, CAN_INQUIRY_PACKET_FRAME_ID);
    frame.can_dlc = static_cast<unsigned char>(CAN_INQUIRY_PACKET_LENGTH);

    // query firmware version
    this->com_data.inquiry_packet.packet_id = CAN_INQUIRY_PACKET_PACKET_ID_FIRMWARE_VERSION_CHOICE;
    can_inquiry_packet_pack(payload, &this->com_data.inquiry_packet, CAN_INQUIRY_PACKET_LENGTH);
    memcpy(frame.data, payload, CAN_INQUIRY_PACKET_LENGTH);

    fv_lock.unlock();

    ssize_t bytes_sent = write(this->can_raw_fd, &frame, static_cast<size_t>(sizeof(frame)));
    if (bytes_sent != static_cast<ssize_t>(sizeof(frame))) {
        throw std::runtime_error("Could not request firmware version!");
    }

    // we should have a response within 1 second
    std::this_thread::sleep_for(1s);

    // query git hash
    this->com_data.inquiry_packet.packet_id = CAN_INQUIRY_PACKET_PACKET_ID_GIT_HASH_CHOICE;
    can_inquiry_packet_pack(payload, &this->com_data.inquiry_packet, CAN_INQUIRY_PACKET_LENGTH);
    memcpy(frame.data, payload, CAN_INQUIRY_PACKET_LENGTH);

    gh_lock.unlock();

    bytes_sent = write(this->can_raw_fd, &frame, static_cast<size_t>(sizeof(frame)));
    if (bytes_sent != static_cast<ssize_t>(sizeof(frame))) {
        throw std::runtime_error("Could not request git hash!");
    }

    // we should have a response within 1 second
    std::this_thread::sleep_for(1s);

    this->print_can_id_info = false;

    // pack firmware info
    fv_lock.lock();
    gh_lock.lock();

    // check if firmware info was really received on RAW thread
    if (current_firmware_version_info.major_version == this->com_data.firmware_version.major_version &&
        current_firmware_version_info.minor_version == this->com_data.firmware_version.minor_version &&
        current_firmware_version_info.build_version == this->com_data.firmware_version.build_version &&
        current_firmware_version_info.platform_type == this->com_data.firmware_version.platform_type &&
        current_firmware_version_info.application_type == this->com_data.firmware_version.application_type
        // ignore Git Hash for now because CPX is not sending it in current firmware implementation
        // (current_git_hash_info.hash_signal == com_data.git_hash.hash_signal)
    ) {
        EVLOG_error << "Could not determine CPX firmware information.";
    }

    this->fw_info =
        std::to_string(can_firmware_version_major_version_decode(this->com_data.firmware_version.major_version)) +
        std::to_string(can_firmware_version_minor_version_decode(this->com_data.firmware_version.minor_version)) +
        std::to_string(can_firmware_version_build_version_decode(this->com_data.firmware_version.build_version)) +
        " (g" + std::to_string(can_git_hash_hash_signal_decode(this->com_data.git_hash.hash_signal)) + ", " +
        std::to_string(can_firmware_version_platform_type_decode(this->com_data.firmware_version.platform_type)) +
        std::to_string(can_firmware_version_application_type_decode(this->com_data.firmware_version.application_type));

    fv_lock.unlock();
    gh_lock.unlock();

    // signal new value
    if (!this->has_cpx_connected_once) {
        EVLOG_info << "Firmware Version and Git Hash received!";
    }
    this->on_fw_info(this->fw_info);
}

void CbCPX::enable() {
    // start sending of periodic Charge Control frames
    this->tx_cc_enabled = true;

    // start reacting on received CAN BCM messages
    this->rx_bcm_enabled = true;

    // start reacting on received CAN RAW messages
    this->rx_raw_enabled = true;
}

canid_t CbCPX::get_can_id(int cpx_id, int message_id) {
    canid_t base_id;

    if (cpx_id <= 0) {
        base_id = static_cast<canid_t>(message_id);
    } else {
        // move cpx_id into upper part and message_id into lower part
        base_id = ((static_cast<canid_t>(cpx_id) & 0x1FFFFF) << 8) | // upper bits
                  (static_cast<canid_t>(message_id) & 0xFF);         // lower bits
    }

    // make sure CAN-ID fits into reserved 29 bits
    base_id &= CAN_EFF_MASK;

    // set extended-frame-flag
    return base_id | CAN_EFF_FLAG;
}

void CbCPX::can_bcm_rx_init() {
    // create msg-buffer
    alignas(std::max(alignof(bcm_msg_head), alignof(can_frame))) std::array<std::byte, can_msg_size> buf {};
    auto* hdr = reinterpret_cast<bcm_msg_head*>(buf.data());
    auto* frame = reinterpret_cast<can_frame*>(buf.data() + sizeof(bcm_msg_head));

    // write CAN-header information for Charge State and PT1000 State
    hdr->opcode = RX_SETUP;
    hdr->flags = RX_FILTER_ID | SETTIMER;
    hdr->nframes = 1;
    hdr->ival1.tv_sec = 0;
    hdr->ival1.tv_usec = 120000;
    hdr->ival2.tv_sec = 0;
    hdr->ival2.tv_usec = 120000;

    // write CAN-header for Charge State
    hdr->can_id = this->get_can_id(this->config.device_id, CAN_CHARGE_STATE1_FRAME_ID);

    // write CAN-frame for Charge State
    frame->can_id = this->get_can_id(this->config.device_id, CAN_CHARGE_STATE1_FRAME_ID);
    frame->can_dlc = static_cast<unsigned char>(CAN_CHARGE_STATE1_LENGTH);
    memset(frame->data, 0x00, CAN_CHARGE_STATE1_LENGTH);

    // init Charge State data for further use
    memset(this->charge_state_data.data(), 0x00, CAN_CHARGE_STATE1_LENGTH);
    this->charge_state_id = this->get_can_id(this->config.device_id, CAN_CHARGE_STATE1_FRAME_ID);

    // init CAN-RX-Filter for Charge State by writing to CAN-Socket
    if (write(this->can_bcm_rx_fd, buf.data(), buf.size()) < 0) {
        throw std::system_error(errno, std::generic_category(), "Charge State RX setup failed!");
    }

    // write CAN-header for PT1000 State
    hdr->can_id = this->get_can_id(this->config.device_id, CAN_PT1000_STATE_FRAME_ID);

    // write CAN-frame for PT1000 State
    frame->can_id = this->get_can_id(this->config.device_id, CAN_PT1000_STATE_FRAME_ID);
    frame->can_dlc = static_cast<unsigned char>(CAN_PT1000_STATE_LENGTH);
    memset(frame->data, 0x00, CAN_PT1000_STATE_LENGTH);

    // init PT1000 State data for further use
    memset(this->pt1000_state_data.data(), 0x00, CAN_PT1000_STATE_LENGTH);
    this->pt1000_state_id = get_can_id(this->config.device_id, CAN_PT1000_STATE_FRAME_ID);

    // init CAN-RX-Filter for PT1000 State by writing to CAN-Socket
    if (write(this->can_bcm_rx_fd, buf.data(), buf.size()) < 0) {
        throw std::system_error(errno, std::generic_category(), "PT1000 State RX setup failed!");
    }
}

void CbCPX::charge_control_update() {
    // create buffer for delete msg
    alignas(std::max(alignof(bcm_msg_head), alignof(can_frame))) std::array<std::byte, can_msg_size> buf_delete {};
    auto* hdr_delete = reinterpret_cast<bcm_msg_head*>(buf_delete.data());

    // create buffer for setup msg
    alignas(std::max(alignof(bcm_msg_head), alignof(can_frame))) std::array<std::byte, can_msg_size> buf_setup {};
    auto* hdr_setup = reinterpret_cast<bcm_msg_head*>(buf_setup.data());
    auto* frame_setup = reinterpret_cast<can_frame*>(buf_setup.data() + sizeof(bcm_msg_head));

    // only update if sending of Charge Control msg is enabled
    if (this->tx_cc_enabled) {
        // only delete previous msg if initialization of Charge Control was done
        if (this->charge_control_initialized) {
            memset(buf_delete.data(), 0, buf_delete.size());
            hdr_delete->opcode = TX_DELETE;
            hdr_delete->can_id = this->get_can_id(this->config.device_id, CAN_CHARGE_CONTROL1_FRAME_ID);
            hdr_delete->nframes = 0;

            // now delete auto sending of Charge Control msg by sending delete msg to socket
            if (write(this->can_bcm_tx_fd, buf_delete.data(), buf_delete.size()) < 0) {
                throw std::system_error(errno, std::generic_category(), "Charge Control delete failed!");
            }
        }

        // define payload variable to pack msg later
        uint8_t payload[8];

        // write CAN-header to setup new Charge Control msg
        hdr_setup->opcode = TX_SETUP;
        hdr_setup->can_id = this->get_can_id(this->config.device_id, CAN_CHARGE_CONTROL1_FRAME_ID);
        hdr_setup->flags = SETTIMER | STARTTIMER;
        hdr_setup->nframes = 1;
        hdr_setup->count = 0;
        hdr_setup->ival1.tv_sec = 0;
        hdr_setup->ival1.tv_usec = 0;
        hdr_setup->ival2.tv_sec = 0;
        hdr_setup->ival2.tv_usec = 100000;

        // take Charge Control mutex to write data
        std::unique_lock<std::mutex> cc_lock(this->cc_mutex);

        // pack Charge Control payload
        can_charge_control1_pack(payload, &this->com_data.charge_control, CAN_CHARGE_CONTROL1_LENGTH);

        // write CAN-frame of new Charge Control msg
        frame_setup->can_id = this->get_can_id(this->config.device_id, CAN_CHARGE_CONTROL1_FRAME_ID);
        frame_setup->len = static_cast<unsigned char>(CAN_CHARGE_CONTROL1_LENGTH);
        memcpy(frame_setup->data, payload, CAN_CHARGE_CONTROL1_LENGTH);

        cc_lock.unlock();

        // setup new Charge Control msg by sending to CAN-socket
        if (write(this->can_bcm_tx_fd, buf_setup.data(), buf_setup.size()) < 0) {
            throw std::system_error(errno, std::generic_category(), "Charge Control init failed!");
        }

        this->charge_control_initialized = true;
    } else {
        throw std::system_error(errno, std::generic_category(), "Enable sending of Charge Control before updating it!");
    }
}

void CbCPX::set_duty_cycle(unsigned int duty_cycle) {
    std::unique_lock<std::mutex> cc_lock(this->cc_mutex);

    this->com_data.charge_control.cc_target_duty_cycle = static_cast<uint16_t>(duty_cycle);
    this->com_data.charge_control.cc_pwm_active = static_cast<uint8_t>(1);

    cc_lock.unlock();

    this->charge_control_update();

    // only check if CPX is not timed out
    // warning about CPX timeout is provided on noticing it
    if (!this->bcm_rx_timeout && this->has_cpx_connected_once) {
        this->launch_duty_cycle_check(duty_cycle);
    }
}

unsigned int CbCPX::get_duty_cycle() {
    return this->get_cs_current_duty_cycle();
}

int CbCPX::switch_state(bool on) {
    std::unique_lock<std::mutex> cc_lock(this->cc_mutex);

    this->com_data.charge_control.cc_contactor1_state = static_cast<uint8_t>(on);
    this->com_data.charge_control.cc_contactor2_state = static_cast<uint8_t>(on);

    cc_lock.unlock();

    this->charge_control_update();

    // if CPX timedout ignore that contactors are not switching
    if (this->bcm_rx_timeout) {
        return 0;
    }

    std::unique_lock<std::mutex> contactor_cv_lock(this->contactor_cv_mutex);
    contactor_change_cv.wait_for(contactor_cv_lock, 1s, [this] { return this->contactor_change.load(); });
    contactor_cv_lock.unlock();

    // // we should see the changes take effect after 1s (FIXME)
    // std::this_thread::sleep_for(1s);

    if (this->get_cs_contactor_state(1) != static_cast<uint8_t>(on)) {
        return 1;
    } else if (this->get_cs_contactor_state(2) != static_cast<uint8_t>(on)) {
        return 2;
    }

    return 0;
}

bool CbCPX::get_contactor_state_no_lock() {
    unsigned int i;
    bool at_least_one_is_configured = false;
    bool target_state = false;
    bool actual_state = false;

    for (i = 1; i <= CB_PROTO_MAX_CONTACTORS; ++i) {
        uint8_t cs_contactor_state_i = this->get_cs_contactor_state(i);

        if (cs_contactor_state_i == CAN_CHARGE_STATE1_CS_CONTACTOR1_STATE_OPEN_CHOICE ||
            cs_contactor_state_i == CAN_CHARGE_STATE1_CS_CONTACTOR1_STATE_CLOSE_CHOICE) {
            at_least_one_is_configured = true;

            // don't overwrite, but merge the state
            actual_state |= static_cast<bool>(cs_contactor_state_i);
        }

        // fallback in the same loop in case no contactor is actually in use
        // don't overwrite, but merge the state
        target_state |= static_cast<bool>(this->get_cc_contactor_state(i));
    }

    if (at_least_one_is_configured)
        return actual_state;
    else
        return target_state;
}

bool CbCPX::get_contactor_state() {
    return this->get_contactor_state_no_lock();
}

unsigned int CbCPX::get_temperature_channels() const {
    return CB_PROTO_MAX_PT1000S;
}

bool CbCPX::is_temperature_enabled(unsigned int channel) {
    return this->get_pt1000_is_active(channel);
}

types::board_support_common::Ampacity CbCPX::pp_state_to_ampacity(uint8_t pp_state) {
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
        EVLOG_error << "PP voltage out of range; no ampacity mapping available.";
        return types::board_support_common::Ampacity::None;
    }
}

types::board_support_common::Ampacity CbCPX::get_ampacity() {
    return this->pp_state_to_ampacity(this->get_cs_current_pp_state());
}

uint16_t CbCPX::get_cs_current_duty_cycle() {
    std::unique_lock<std::mutex> cs_lock(this->cs_mutex);
    return this->com_data.charge_state.cs_current_duty_cycle;
}

uint8_t CbCPX::get_cs_pwm_active() {
    std::unique_lock<std::mutex> cs_lock(this->cs_mutex);
    return this->com_data.charge_state.cs_pwm_active;
}

uint8_t CbCPX::get_cs_current_cp_state() {
    std::unique_lock<std::mutex> cs_lock(this->cs_mutex);
    return this->com_data.charge_state.cs_current_cp_state;
}

uint8_t CbCPX::get_cs_short_circuit() {
    std::unique_lock<std::mutex> cs_lock(this->cs_mutex);
    return this->com_data.charge_state.cs_cp_short_circuit;
}

uint8_t CbCPX::get_cs_diode_fault() {
    std::unique_lock<std::mutex> cs_lock(this->cs_mutex);
    return this->com_data.charge_state.cs_diode_fault;
}

uint8_t CbCPX::get_cs_current_pp_state() {
    std::unique_lock<std::mutex> cs_lock(this->cs_mutex);
    return this->com_data.charge_state.cs_current_pp_state;
}

uint8_t CbCPX::get_cs_contactor_state(int contactor) {
    std::unique_lock<std::mutex> cs_lock(this->cs_mutex);
    switch (contactor) {
    case 1:
        return this->com_data.charge_state.cs_contactor1_state;

    case 2:
        return this->com_data.charge_state.cs_contactor2_state;

    default:
        throw std::system_error(errno, std::generic_category(), "Selected contactor out of range!");
    }
}

uint8_t CbCPX::get_cc_contactor_state(int contactor) {
    std::unique_lock<std::mutex> cc_lock(this->cc_mutex);
    switch (contactor) {
    case 1:
        return this->com_data.charge_control.cc_contactor1_state;

    case 2:
        return this->com_data.charge_control.cc_contactor2_state;

    default:
        throw std::system_error(errno, std::generic_category(), "Selected contactor out of range!");
    }
}

bool CbCPX::is_cs_contactor_error(int contactor) {
    std::unique_lock<std::mutex> cs_lock(this->cs_mutex);
    switch (contactor) {
    case 1:
        return static_cast<bool>(this->com_data.charge_state.cs_contactor1_error);

    case 2:
        return static_cast<bool>(this->com_data.charge_state.cs_contactor2_error);

    case 0:
        return static_cast<bool>(this->com_data.charge_state.cs_contactor1_error ||
                                 this->com_data.charge_state.cs_contactor2_error);

    default:
        throw std::system_error(errno, std::generic_category(), "Selected contactor out of range!");
    }
}

uint8_t CbCPX::get_cs_hv_ready() {
    std::unique_lock<std::mutex> cs_lock(this->cs_mutex);
    return this->com_data.charge_state.cs_hv_ready;
}

bool CbCPX::is_cs_estop_charging_abort(int estop) {
    std::unique_lock<std::mutex> cs_lock(this->cs_mutex);
    switch (estop) {
    case 1:
        return static_cast<bool>(this->com_data.charge_state.cs_estop1_charging_abort == 1);

    case 2:
        return static_cast<bool>(this->com_data.charge_state.cs_estop2_charging_abort == 1);

    case 3:
        return static_cast<bool>(this->com_data.charge_state.cs_estop3_charging_abort == 1);

    case 0:
        return static_cast<bool>(this->com_data.charge_state.cs_estop1_charging_abort == 1 ||
                                 this->com_data.charge_state.cs_estop2_charging_abort == 1 ||
                                 this->com_data.charge_state.cs_estop3_charging_abort == 1);

    default:
        throw std::system_error(errno, std::generic_category(), "Selected estop out of range!");
    }
}

bool CbCPX::get_pt1000_is_active(int channel) {
    std::unique_lock<std::mutex> pt_lock(this->pt_mutex);
    switch (channel) {
    case 1:
        return static_cast<bool>(this->com_data.pt1000_state.pt1_temperature !=
                                 CAN_PT1000_STATE_PT1_TEMPERATURE_TEMP_SENSOR_NOT_USED_CHOICE);

    case 2:
        return static_cast<bool>(this->com_data.pt1000_state.pt2_temperature !=
                                 CAN_PT1000_STATE_PT2_TEMPERATURE_TEMP_SENSOR_NOT_USED_CHOICE);

    case 3:
        return static_cast<bool>(this->com_data.pt1000_state.pt3_temperature !=
                                 CAN_PT1000_STATE_PT3_TEMPERATURE_TEMP_SENSOR_NOT_USED_CHOICE);

    case 4:
        return static_cast<bool>(this->com_data.pt1000_state.pt4_temperature !=
                                 CAN_PT1000_STATE_PT4_TEMPERATURE_TEMP_SENSOR_NOT_USED_CHOICE);

    default:
        throw std::system_error(errno, std::generic_category(), "Selected PT1000 channel out of range!");
    }
}

bool CbCPX::is_temperature_valid(unsigned int channel) {
    std::unique_lock<std::mutex> pt_lock(this->pt_mutex);
    switch (channel) {
    case 1:
        return static_cast<bool>(
            !(this->com_data.pt1000_state.pt1_selftest_failed && this->com_data.pt1000_state.pt1_charging_stopped));

    case 2:
        return static_cast<bool>(
            !(this->com_data.pt1000_state.pt2_selftest_failed && this->com_data.pt1000_state.pt2_charging_stopped));

    case 3:
        return static_cast<bool>(
            !(this->com_data.pt1000_state.pt3_selftest_failed && this->com_data.pt1000_state.pt3_charging_stopped));

    case 4:
        return static_cast<bool>(
            !(this->com_data.pt1000_state.pt4_selftest_failed && this->com_data.pt1000_state.pt4_charging_stopped));

    default:
        throw std::system_error(errno, std::generic_category(), "Selected PT1000 channel out of range!");
    }
}

bool CbCPX::is_pt_selftest_failed(unsigned int channel) {
    std::unique_lock<std::mutex> pt_lock(this->pt_mutex);
    switch (channel) {
    case 1:
        return static_cast<bool>(this->com_data.pt1000_state.pt1_selftest_failed);

    case 2:
        return static_cast<bool>(this->com_data.pt1000_state.pt2_selftest_failed);

    case 3:
        return static_cast<bool>(this->com_data.pt1000_state.pt3_selftest_failed);

    case 4:
        return static_cast<bool>(this->com_data.pt1000_state.pt4_selftest_failed);

    case 0:
        return static_cast<bool>(
            this->com_data.pt1000_state.pt1_selftest_failed || this->com_data.pt1000_state.pt2_selftest_failed ||
            this->com_data.pt1000_state.pt3_selftest_failed || this->com_data.pt1000_state.pt4_selftest_failed);

    default:
        throw std::system_error(errno, std::generic_category(), "Selected PT1000 channel out of range!");
    }
}

bool CbCPX::is_pt_charging_stopped(unsigned int channel) {
    std::unique_lock<std::mutex> pt_lock(this->pt_mutex);
    switch (channel) {
    case 1:
        return static_cast<bool>(this->com_data.pt1000_state.pt1_charging_stopped);

    case 2:
        return static_cast<bool>(this->com_data.pt1000_state.pt2_charging_stopped);

    case 3:
        return static_cast<bool>(this->com_data.pt1000_state.pt3_charging_stopped);

    case 4:
        return static_cast<bool>(this->com_data.pt1000_state.pt4_charging_stopped);

    case 0:
        return static_cast<bool>(
            this->com_data.pt1000_state.pt1_charging_stopped || this->com_data.pt1000_state.pt2_charging_stopped ||
            this->com_data.pt1000_state.pt3_charging_stopped || this->com_data.pt1000_state.pt4_charging_stopped);

    default:
        throw std::system_error(errno, std::generic_category(), "Selected PT1000 channel out of range!");
    }
}

float CbCPX::get_temperature(unsigned int channel) {
    std::unique_lock<std::mutex> pt_lock(this->pt_mutex);
    switch (channel) {
    case 1:
        return static_cast<float>(this->com_data.pt1000_state.pt1_temperature);

    case 2:
        return static_cast<float>(this->com_data.pt1000_state.pt2_temperature);

    case 3:
        return static_cast<float>(this->com_data.pt1000_state.pt3_temperature);

    case 4:
        return static_cast<float>(this->com_data.pt1000_state.pt4_temperature);

    default:
        throw std::system_error(errno, std::generic_category(), "Selected PT1000 channel out of range!");
    }
}

bool CbCPX::is_emergency() {
    bool pp_error = false;

    if (this->is_pluggable) {
        // any state above this is considered an error for now
        pp_error = this->get_cs_current_pp_state() > CAN_CHARGE_STATE1_CS_CURRENT_PP_STATE_63_70_A_CHOICE;
    }

    return pp_error || (this->get_cs_current_cp_state() == CAN_CHARGE_STATE1_CS_CURRENT_CP_STATE_INVALID_CHOICE) ||
           (this->get_cs_short_circuit() != 0) || (this->get_cs_diode_fault() != 0) || this->is_cs_contactor_error() ||
           this->is_cs_estop_charging_abort() || this->is_pt_selftest_failed() || this->is_pt_charging_stopped();
}

void CbCPX::read_charge_state(uint8_t* data) {
    std::unique_lock<std::mutex> cs_lock(this->cs_mutex);

    can_charge_state1_unpack(&this->com_data.charge_state, data, CAN_CHARGE_STATE1_LENGTH);

    cs_lock.unlock();
}

void CbCPX::read_pt1000_state(uint8_t* data) {
    std::unique_lock<std::mutex> pt_lock(this->pt_mutex);

    can_pt1000_state_unpack(&this->com_data.pt1000_state, data, CAN_PT1000_STATE_LENGTH);

    // multiply temperatures with correct scale
    this->com_data.pt1000_state.pt1_temperature *= static_cast<double>(CB_PROTO_TEMP_SCALE);
    this->com_data.pt1000_state.pt2_temperature *= static_cast<double>(CB_PROTO_TEMP_SCALE);
    this->com_data.pt1000_state.pt3_temperature *= static_cast<double>(CB_PROTO_TEMP_SCALE);
    this->com_data.pt1000_state.pt4_temperature *= static_cast<double>(CB_PROTO_TEMP_SCALE);

    pt_lock.unlock();
}

void CbCPX::read_fw_version(uint8_t* data) {
    std::unique_lock<std::mutex> fv_lock(this->fv_mutex);
    can_firmware_version_unpack(&this->com_data.firmware_version, data, CAN_FIRMWARE_VERSION_LENGTH);
}

void CbCPX::read_git_hash(uint8_t* data) {
    std::unique_lock<std::mutex> gh_lock(this->gh_mutex);
    can_git_hash_unpack(&this->com_data.git_hash, data, CAN_GIT_HASH_LENGTH);
}

bool CbCPX::is_any_notify_flag_set() {
    return notify_flags.any();
}

bool CbCPX::is_contactor_error(int contactor) {
    uint8_t cs_contactor_state = this->get_cs_contactor_state(contactor);

    return (cs_contactor_state == CAN_CHARGE_STATE1_CS_CONTACTOR1_STATE_OPEN_CHOICE ||
            cs_contactor_state == CAN_CHARGE_STATE1_CS_CONTACTOR1_STATE_CLOSE_CHOICE) &&
           this->is_cs_contactor_error(contactor);
}

void CbCPX::launch_duty_cycle_check(unsigned int expected_duty_cycle) {
    // stop previous thread if running
    if (this->duty_cycle_check_thread.joinable()) {
        this->duty_cycle_check_termination_requested = true;
        this->duty_cycle_check_cv.notify_all();
        this->duty_cycle_check_thread.join();
    }

    this->duty_cycle_check_termination_requested = false;

    this->duty_cycle_check_thread = std::thread([this, expected_duty_cycle]() {
        std::unique_lock<std::mutex> lock(this->duty_cycle_check_mutex);
        while (!this->duty_cycle_check_termination_requested) {
            const bool notified = this->duty_cycle_check_cv.wait_for(
                lock, 1s, [this] { return this->duty_cycle_check_termination_requested.load(); });

            if (this->duty_cycle_check_termination_requested || notified)
                break;

            const double duty_cycle = expected_duty_cycle / 10.0;
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(1) << duty_cycle;

            if (expected_duty_cycle == this->get_cs_current_duty_cycle()) {
                EVLOG_debug << "[suppressed] Safety Controller did accept the new duty cycle of " << oss.str() << "%";
            } else {
                EVLOG_warning << "Safety Controller did not accept the new duty cycle of " << oss.str() << "%";
            }

            break;
        }
    });
}

void CbCPX::handle_timeout_watchdog(int flag) {
    // disable thread in case CPX reports back
    if (flag == 0) {
        this->timeout_watchdog_termination_requested = true;
        this->timeout_watchdog_cv.notify_all();
        if (this->timeout_watchdog_thread.joinable()) {
            this->timeout_watchdog_thread.join();
        }
        this->timeout_watchdog_termination_requested = false;
        this->bcm_rx_timeout = false;
        EVLOG_info << "Receiving on BCM socket after timeout (CPX-ID: " << std::to_string(this->config.device_id)
                   << ").";
        this->on_cpx_timeout(false);
        return;
    }

    if (this->timeout_watchdog_thread.joinable()) {
        this->timeout_watchdog_termination_requested = true;
        this->timeout_watchdog_cv.notify_all();
        this->timeout_watchdog_thread.join();
    }

    this->timeout_watchdog_termination_requested = false;

    this->timeout_watchdog_thread = std::thread([this] {
        std::unique_lock<std::mutex> lock(this->timeout_watchdog_mutex);
        while (!this->timeout_watchdog_termination_requested) {
            bool notified =
                this->timeout_watchdog_cv.wait_for(lock, std::chrono::seconds(this->cpx_timeout_seconds), [this] {
                    return this->timeout_watchdog_termination_requested.load();
                });

            if (this->timeout_watchdog_termination_requested || notified)
                break;

            if (this->bcm_rx_timeout) {
                EVLOG_warning << "Failed to receive on BCM socket - CPX-ID: " << std::to_string(this->config.device_id);
                this->on_cpx_timeout(true);
                break;
            }
        }
    });
}

void CbCPX::notify_worker() {
    uint8_t previous_cp_state = static_cast<uint8_t>(CAN_CHARGE_STATE1_CS_CURRENT_CP_STATE_INVALID_CHOICE + 1);

    uint8_t current_cp_state;
    uint8_t current_pp_state;

    while (!this->termination_requested) {
        // react to changes relevant for notification
        std::unique_lock<std::mutex> lock(this->notify_mutex);

        this->notify_worker_cv.wait(lock,
                                    [&] { return this->termination_requested || this->is_any_notify_flag_set(); });

        const bool pp_changed = notify_exchange(NotifyFlag::PpChanged, false);
        const bool cp_changed = notify_exchange(NotifyFlag::CpChanged, false);
        const bool estop_1_changed = notify_exchange(NotifyFlag::Estop1Changed, false);
        const bool estop_2_changed = notify_exchange(NotifyFlag::Estop2Changed, false);
        const bool estop_3_changed = notify_exchange(NotifyFlag::Estop3Changed, false);
        const bool cp_error = notify_exchange(NotifyFlag::CpError, false);
        const bool contactor_1_error = notify_exchange(NotifyFlag::Contactor1Error, false);
        const bool contactor_2_error = notify_exchange(NotifyFlag::Contactor2Error, false);
        lock.unlock();

        // check for PP changes
        if (pp_changed) {
            current_pp_state = this->get_cs_current_pp_state();
            if (this->is_pluggable) {
                EVLOG_debug << "on_pp_change(" << this->pp_state_to_ampacity(current_pp_state) << ")";
                this->on_pp_change(current_pp_state);
            } else {
                EVLOG_debug << "on_pp_change(" << this->pp_state_to_ampacity(current_pp_state)
                            << ") [suppressed, fixed cable]";
            }
        }

        // check for CP changes
        if (cp_changed) {
            current_cp_state = this->get_cs_current_cp_state();
            if (previous_cp_state != CAN_CHARGE_STATE1_CS_CURRENT_CP_STATE_UNKNOWN_CHOICE) {
                EVLOG_debug << "on_cp_change(" << static_cast<types::cb_board_support::CPState>(current_cp_state)
                            << ")";
                this->on_cp_change(current_cp_state);
            } else {
                EVLOG_debug << "on_cp_change(" << static_cast<types::cb_board_support::CPState>(current_cp_state)
                            << ") [suppressed]";
            }
            previous_cp_state = current_cp_state;
        }

        // check for ESTOPs
        if (estop_1_changed) {
            this->on_estop(1u, this->is_cs_estop_charging_abort(1));
        }

        if (estop_2_changed) {
            this->on_estop(2u, this->is_cs_estop_charging_abort(2));
        }

        if (estop_3_changed) {
            this->on_estop(3u, this->is_cs_estop_charging_abort(3));
        }

        // check for contactor errors
        if (contactor_1_error) {
            const std::string name = "Contactor 1";
            this->on_contactor_error(name, static_cast<bool>(this->get_cc_contactor_state(1)),
                                     static_cast<bool>(this->get_cs_contactor_state(1))
                                         ? types::cb_board_support::ContactorState::Closed
                                         : types::cb_board_support::ContactorState::Open);
        }

        if (contactor_2_error) {
            const std::string name = "Contactor 2";
            this->on_contactor_error(name, static_cast<bool>(this->get_cc_contactor_state(2)),
                                     static_cast<bool>(this->get_cs_contactor_state(2))
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

void CbCPX::can_bcm_rx_worker() {
    // create buffer to handle CAN-RX-msg
    alignas(std::max(alignof(bcm_msg_head), alignof(can_frame))) std::array<std::byte, can_msg_size> buf {};
    auto* hdr = reinterpret_cast<bcm_msg_head*>(buf.data());
    auto* frame = reinterpret_cast<can_frame*>(buf.data() + sizeof(bcm_msg_head));

    // Snapshot of CAN-derived states used to decide which notify flags must fire.
    struct NotifyState {
        uint8_t pp_state {0};
        uint8_t cp_state {0};
        bool estop[CB_PROTO_MAX_ESTOPS] {false, false, false};
        unsigned int cp_errors {0};
        bool contactor_errors[CB_PROTO_MAX_CONTACTORS] {false, false};
        bool contactor_states[CB_PROTO_MAX_CONTACTORS] {false, false};
    };

    // Capture the current notification state so we can compare before/after decoding a frame.
    auto capture_notify_state = [this]() {
        NotifyState state;
        state.pp_state = this->get_cs_current_pp_state();
        state.cp_state = this->get_cs_current_cp_state();
        for (unsigned int i = 0; i < CB_PROTO_MAX_ESTOPS; ++i) {
            state.estop[i] = this->is_cs_estop_charging_abort(i + 1);
        }
        state.cp_errors = (static_cast<unsigned int>(this->get_cs_diode_fault()) << 1) |
                          static_cast<unsigned int>(this->get_cs_short_circuit());
        for (unsigned int i = 0; i < CB_PROTO_MAX_CONTACTORS; ++i) {
            state.contactor_errors[i] = this->is_contactor_error(i + 1);
            state.contactor_states[i] = static_cast<bool>(this->get_cs_contactor_state(i + 1));
        }
        return state;
    };

    EVLOG_info << "CAN BCM Rx Thread started";

    while (!this->termination_requested) {
        if (this->rx_bcm_enabled) {
            const ssize_t rv = read(this->can_bcm_rx_fd, buf.data(), buf.size());

            if (rv < 0) {
                EVLOG_warning << "Read from Rx BCM socket failed.";
            }
            if (rv == 0)
                continue; // this should usually not happen
            // we should receive either a header plus frame, or only the header in case of RX_TIMEOUT
            if (rv < static_cast<ssize_t>(sizeof(bcm_msg_head) + hdr->nframes * sizeof(can_frame))) {
                EVLOG_warning << "Short CAN message read on Rx BCM socket.";
            }

            // check for timeout or received data
            if (hdr->opcode == RX_TIMEOUT) {
                this->bcm_rx_timeout = true;
                this->handle_timeout_watchdog(1);

            } else if (hdr->opcode == RX_CHANGED) {
                // compare current and received data by CAN-ID
                if (hdr->can_id == this->charge_state_id &&
                    memcmp(frame->data, this->charge_state_data.data(), 8) != 0) {
                    // remember new Charge State data as current
                    std::memcpy(this->charge_state_data.data(), frame->data, CAN_MAX_DLEN);

                    // remember current notify state
                    const auto previous_notify_state = capture_notify_state();

                    this->read_charge_state(frame->data);

                    // remember new notify state
                    const auto new_notify_state = capture_notify_state();

                    bool notify_required = false;

                    auto mark_change = [&](bool changed, NotifyFlag flag) {
                        if (changed) {
                            this->notify_set(flag, true);
                            notify_required = true;
                        }
                    };

                    {
                        std::lock_guard<std::mutex> lock(this->notify_mutex);

                        mark_change(previous_notify_state.pp_state != new_notify_state.pp_state, NotifyFlag::PpChanged);
                        mark_change(previous_notify_state.cp_state != new_notify_state.cp_state, NotifyFlag::CpChanged);
                        mark_change(previous_notify_state.cp_errors != new_notify_state.cp_errors, NotifyFlag::CpError);
                        mark_change(previous_notify_state.contactor_errors[0] != new_notify_state.contactor_errors[0],
                                    NotifyFlag::Contactor1Error);
                        mark_change(previous_notify_state.contactor_errors[1] != new_notify_state.contactor_errors[1],
                                    NotifyFlag::Contactor2Error);
                        mark_change(previous_notify_state.estop[0] != new_notify_state.estop[0],
                                    NotifyFlag::Estop1Changed);
                        mark_change(previous_notify_state.estop[1] != new_notify_state.estop[1],
                                    NotifyFlag::Estop2Changed);
                        mark_change(previous_notify_state.estop[2] != new_notify_state.estop[2],
                                    NotifyFlag::Estop3Changed);
                    }

                    if (notify_required) {
                        this->notify_worker_cv.notify_one();
                    }

                    // check contactor states seperately because it is only used within this module and no flag to be
                    // read from the outside is set
                    std::lock_guard<std::mutex> contactor_cv_lock(this->contactor_cv_mutex);
                    this->contactor_change = false;
                    if (previous_notify_state.contactor_states[0] != new_notify_state.contactor_states[0] ||
                        previous_notify_state.contactor_states[1] != new_notify_state.contactor_states[1]) {
                        this->contactor_change = true;
                        this->contactor_change_cv.notify_one();
                    }

                } else if (hdr->can_id == this->pt1000_state_id &&
                           memcmp(frame->data, this->pt1000_state_data.data(), 8) != 0) {
                    // remember new frame as current
                    memcpy(this->pt1000_state_data.data(), frame->data, CAN_MAX_DLEN);

                    // make new received data available
                    this->read_pt1000_state(frame->data);
                    this->temperature_data_is_valid = true;

                } else if (hdr->can_id != this->charge_state_id && hdr->can_id != this->pt1000_state_id) {
                    EVLOG_info << "[RECV] CAN ID: 0x" << std::hex << hdr->can_id;
                    EVLOG_info << "[Charge State] CAN ID: 0x" << std::hex << this->charge_state_id;
                    EVLOG_info << "[PT1000 State] CAN ID: 0x" << std::hex << this->pt1000_state_id;
                    EVLOG_warning << "CAN BCM RX: Unknown CAN-ID received";
                }

                if (hdr->can_id == this->charge_state_id || hdr->can_id == this->pt1000_state_id) {
                    // do stuff in case CPX was not yet connected
                    if (!this->has_cpx_connected_once) {
                        // request firmware version and git hash on first connection
                        this->get_firmware_and_git_hash();

                        // check if CPX accepted duty cycle
                        this->launch_duty_cycle_check(this->com_data.charge_control.cc_target_duty_cycle);

                        this->has_cpx_connected_once = true;
                    }

                    // if expected CAN-ID was received and timeout active
                    // reset timeout flag and report back
                    if (this->bcm_rx_timeout == true) {
                        this->handle_timeout_watchdog(0);
                    }
                }
            } else {
                EVLOG_warning << "Unexpected BCM opcode received: " << std::to_string(hdr->opcode);
            }
        }
    }

    EVLOG_info << "CAN BCM Rx Thread stopped";
}

void CbCPX::can_raw_rx_worker() {
    struct can_frame frame;

    EVLOG_info << "CAN RAW Rx Thread started";

    while (!this->termination_requested) {
        if (this->rx_raw_enabled) {
            // read blocks until new frame is available
            const ssize_t rv = read(this->can_raw_fd, &frame, sizeof(frame));
            if (rv < 0) {
                EVLOG_warning << "Read from RAW socket failed.";
            }
            if (rv == 0 || rv < static_cast<ssize_t>(sizeof(struct can_frame)))
                continue; // this should usually not happen

            if (frame.can_id == this->get_can_id(this->config.device_id, CAN_FIRMWARE_VERSION_FRAME_ID)) {
                this->read_fw_version(frame.data);
            }

            if (frame.can_id == this->get_can_id(this->config.device_id, CAN_GIT_HASH_FRAME_ID)) {
                this->read_git_hash(frame.data);
            }
        }
    }

    EVLOG_info << "CAN RAW Rx Thread stopped";
}
