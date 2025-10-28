// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include "InfypowerCANController.hpp"

#include <cerrno>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iosfwd>
#include <iostream>
#include <string>
#include <stdexcept>
#include <system_error>
#include <utility>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/bcm.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <can_netlink.h>
#include <unistd.h>
#include <libsocketcan.h>
#include <everest/logging.hpp>
#include "datatype_tools.h"
#include "InfypowerCANCmd.hpp"

InfypowerCANController::InfypowerCANController() {
    // we can only pre-init some of the caps statically, others will be filled later
    // after communication is up or we know the configuration
    //
    // clang-format off
    // struct Capabilities {
    //     bool bidirectional; -- true: support bidirectional energy flow, false: support only Export mode (output)
    //     float current_regulation_tolerance_A; -- Absolute magnitude of the regulation tolerance in Ampere
    //     float peak_current_ripple_A; -- Peak-to-peak magnitude of the current ripple in Ampere
    //     float max_export_voltage_V; -- Maximum voltage that the power supply can output in Volt
    //     float min_export_voltage_V; -- Minimum voltage that the power supply can output in Volt
    //     float max_export_current_A; -- Maximum current that the power supply can output in Ampere
    //     float min_export_current_A; -- Minimum current limit that the power supply can set in Ampere
    //     float max_export_power_W; -- Maximum export power that the power supply can output in Watt
    //     std::optional<float> max_import_voltage_V; -- Maximum voltage that the power supply supports to import energy in Volt
    //     std::optional<float> min_import_voltage_V; -- Minimum voltage that the power supply requires to import energy in Volt
    //     std::optional<float> max_import_current_A; -- Maximum current that the power supply can output in Ampere
    //     std::optional<float> min_import_current_A; -- Minimum current limit that the power supply can set in Ampere
    //     std::optional<float> max_import_power_W; -- Maximum import power that the power supply can sink in Watt
    //     std::optional<float> conversion_efficiency_import; -- Typical import efficiency used for energy management
    //     std::optional<float> conversion_efficiency_export; -- Typical export efficiency used for energy management
    // };
    // clang-format on
    this->caps.current_regulation_tolerance_A = 2.0f; // TODO: clarify source of this value
    this->caps.peak_current_ripple_A = 2.0f;          // TODO: clarify source of this value
    this->caps.conversion_efficiency_import = 0.85f;  // TODO: clarify source of this value, maybe adjust dynamically
    this->caps.conversion_efficiency_export = 0.9f;   // TODO: clarify source of this value, maybe adjust dynamically
}

InfypowerCANController::~InfypowerCANController() {
    // request termination of our worker threads
    this->termination_requested = true;

    // if thread is active wait until it is terminated
    if (this->can_bcm_thread.joinable())
        this->can_bcm_thread.join();

    // if thread is active wait until it is terminated
    if (this->can_raw_thread.joinable())
        this->can_raw_thread.join();

    // close sockets
    if (this->can_bcm_fd != -1)
        close(this->can_bcm_fd);

    if (this->can_raw_fd != -1)
        close(this->can_raw_fd);
}

void InfypowerCANController::init(const std::string& device, unsigned int bitrate, unsigned int can_source_address,
                                  unsigned int can_destination_address, const std::string& dc_module_type) {
    struct can_bittiming bt;
    struct sockaddr_can addr;
    struct ifreq ifr;
    int state;

    // remember various parameters
    this->device = device;
    this->can_src_addr = can_source_address;
    this->can_dst_addr = can_destination_address;
    this->dc_module_type = dc_module_type;

    // check and remember whether the DC power module is a bidirectional one (based on user configuration input)
    auto it = std::find(this->bidi_dc_module_types.begin(), this->bidi_dc_module_types.end(), dc_module_type);
    this->caps.bidirectional = it != this->bidi_dc_module_types.end();
    EVLOG_info << "Bidirectional capability: " << std::boolalpha << this->caps.bidirectional;

    // get current interface configuration and state
    if (can_get_state(device.c_str(), &state))
        throw std::system_error(errno, std::generic_category(), "Failed to open '" + device + "'");

    if (can_get_bittiming(device.c_str(), &bt))
        throw std::system_error(errno, std::generic_category(),
                                "Failed to retrieve current bitrate of '" + device + "'");

    // check whether the current bitrate is different from desired one and if so try to change it
    if (bt.bitrate != bitrate) {
        // stop the interface first if already running
        if (state != CAN_STATE_STOPPED) {
            if (can_do_stop(device.c_str()))
                throw std::system_error(errno, std::generic_category(),
                                        "Could not stop '" + device + "' before re-configuring");
            state = CAN_STATE_STOPPED;
        }

        if (can_set_bitrate(device.c_str(), bitrate))
            throw std::system_error(errno, std::generic_category(),
                                    "Could not re-configure bitrate on '" + device + "'");
    }

    // if interface is not running (anymore) try to (re-)start it
    if (state == CAN_STATE_STOPPED) {
        if (can_do_start(device.c_str()))
            throw std::system_error(errno, std::generic_category(), "Could not start '" + device + "'");
    }

    // open a CAN BCM socket, bound to the desired interface
    this->can_bcm_fd = socket(PF_CAN, SOCK_DGRAM, CAN_BCM);
    if (this->can_bcm_fd == -1)
        throw std::system_error(errno, std::generic_category(), "socket(PF_CAN, CAN_BCM) failed");

    strncpy(ifr.ifr_name, device.c_str(), sizeof(ifr.ifr_name));
    if (ioctl(this->can_bcm_fd, SIOCGIFINDEX, &ifr))
        throw std::system_error(errno, std::generic_category(),
                                "Couldn't determine interface number of '" + device + "'");

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (connect(this->can_bcm_fd, (struct sockaddr*)&addr, sizeof(addr)))
        throw std::system_error(errno, std::generic_category(), "Couldn't connect CAN BCM socket on '" + device + "'");

    // open a CAN RAW socket, bound to the desired interface
    this->can_raw_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (this->can_raw_fd == -1)
        throw std::system_error(errno, std::generic_category(), "socket(PF_CAN, CAN_RAW) failed");

    if (bind(this->can_raw_fd, (struct sockaddr*)&addr, sizeof(addr)))
        throw std::system_error(errno, std::generic_category(), "Couldn't bind CAN RAW socket on '" + device + "'");

    // now we have already set up two generic CAN sockets
    // let's call a power module specific CAN setup method for detailed CAN frame setup
    // (can be later different for different power module types)
    this->pm_can_setup();

    // launch CAN RAW rx thread
    this->can_raw_thread = std::thread(&InfypowerCANController::can_raw_rx_worker, this);

    // launch CAN BCM rx thread
    this->can_bcm_thread = std::thread(&InfypowerCANController::can_bcm_rx_worker, this);

    // this must be the first command since we need to know how many devices are in our group
    this->pm_query_count();

    // safety first: disable all modules in the group now that we know how many we have
    this->set_enable(false);

    // query the PM caps
    this->pm_query_caps();

    // set initial working mode to export (rectifier mode)
    this->set_import_mode(false);
}

void InfypowerCANController::pm_can_setup() {
    // CAN ID used when setting working mode, voltage and current and switch on/off
    this->can_id_write_cfg = InfypowerTxCANID(this->can_src_addr, this->can_dst_addr, cmd::write_cfg);

    // setup filter for CAN RAW messages
    this->setup_can_raw();

    // register CAN BCM messages
    this->setup_can_bcm();
}

void InfypowerCANController::setup_can_bcm() {
// suppress warning: missing initializer for member ‘bcm_msg_head::frames’ [-Wmissing-field-initializers]
// it is none-sense since the field is array of length zero and thus cannot be initialized (actually, it is
// with the following struct can_frame...)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

    // we use 0 expected responses here, since it is actually infinite

    auto cmd_v = std::make_unique<InfypowerCANCmd>(
        "Read System DC Voltage", InfypowerTxCANID(this->can_src_addr, this->can_dst_addr, cmd::read_info), 0x10, 0x01,
        0);
    cmd_v->on_received.connect([this](struct can_frame* can_frame) {
        this->received_voltage = static_cast<int32_t>(extract_uint32(&can_frame->data[4]));

        // Note: Everest expects voltage and current in a single update, but we have two CAN frames here,
        // so we don't do anything here but wait until the frame with the current value is received
    });
    this->can_bcm_cmds.push_back(std::move(cmd_v));

    auto cmd_c = std::make_unique<InfypowerCANCmd>(
        "Read System DC Current", InfypowerTxCANID(this->can_src_addr, this->can_dst_addr, cmd::read_info), 0x10, 0x02,
        0);
    cmd_c->on_received.connect([this](struct can_frame* can_frame) {
        this->received_current = static_cast<int32_t>(extract_uint32(&can_frame->data[4]));

        // now we have the current and use the cached voltage value
        float v = std::abs(static_cast<float>(this->received_voltage) / 1000.0f);
        float c = std::abs(static_cast<float>(this->received_current) / 1000.0f);
        this->on_vc_update(v, c);
    });
    this->can_bcm_cmds.push_back(std::move(cmd_c));

    this->can_bcm_cmds.push_back(std::make_unique<InfypowerCANCmd>(
        "Read Power Module Status", InfypowerTxCANID(this->can_src_addr, this->can_dst_addr, cmd::read_info), 0x11,
        0x10, 0));

    // register these commands in our expectation list
    for (auto it = this->can_bcm_cmds.begin(); it != this->can_bcm_cmds.end(); ++it)
        this->register_expected_cmd(**it);

    // the CAN ID is the same for all commands here, so we just use this below
    const InfypowerCANCmd& cmd = *this->can_bcm_cmds[0];

    // we cannot use the InfypowerRxCANID provided by InfypowerCANCmd because it has narrowed mask,
    // so we have to create it for our use-case here: just swapped but complete
    const InfypowerRxCANID can_id_rx(cmd.get_tx_can_id());

    // we actually only register Rx for Read Power Module Status here since we have to
    // periodically report voltage and current to EVerest
    char bcm_rx_setup[sizeof(struct bcm_msg_head) + 2 * sizeof(struct can_frame)] = {};
    struct bcm_msg_head* bcm_msg_head = (struct bcm_msg_head*)bcm_rx_setup;
    struct can_frame* can_frame = bcm_msg_head->frames;
    *bcm_msg_head = (struct bcm_msg_head) {
        .opcode = RX_SETUP,
        .flags = SETTIMER | RX_ANNOUNCE_RESUME,
        .count = 0,
        .ival1 =
            {
                .tv_sec = 0,
                .tv_usec = 385000, // expect frames in intervals of 375 ms plus 10 ms safety margin
            },
        .ival2 =
            {
                .tv_sec = 0,
                .tv_usec = 250000, // limit updates to once every 250ms (see EVerest's recommendation)
            },
        .can_id = can_id_rx,
        .nframes = 2,
    };
    can_frame[0] = (struct can_frame) {
        .can_id = can_id_rx,
        .len = 8,
        // MUX mask
        .data = {0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    };
    can_frame[1] = (struct can_frame) {
        .can_id = can_id_rx,
        .len = 8,
        // mask for power module status
        .data = {0x11, 0x10, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff},
    };

    // push the setup request to CAN BCM
    this->push_to_can_bcm(bcm_rx_setup, sizeof(bcm_rx_setup), "Couldn't register response frames with BCM");

    char bcm_tx_setup[sizeof(struct bcm_msg_head) + 3 * sizeof(struct can_frame)] = {};
    bcm_msg_head = (struct bcm_msg_head*)bcm_tx_setup;
    can_frame = bcm_msg_head->frames;

    bcm_msg_head->opcode = TX_SETUP;
    bcm_msg_head->flags = SETTIMER | STARTTIMER | TX_CP_CAN_ID;
    // send at intervals of 125 ms alternating 3 frames -> effective cycle is 375ms,
    // 250ms for voltage and current as recommended by EVerest
    bcm_msg_head->ival2.tv_usec = 125000;
    bcm_msg_head->can_id = cmd.get_tx_can_id();
    bcm_msg_head->nframes = this->can_bcm_cmds.size();

    for (unsigned int idx = 0; idx < this->can_bcm_cmds.size(); ++idx)
        memcpy(&can_frame[idx], this->can_bcm_cmds[idx]->get_tx_frame(), sizeof(struct can_frame));

    // push the setup request to CAN BCM
    this->push_to_can_bcm(bcm_tx_setup, sizeof(bcm_tx_setup), "Couldn't register request frames with BCM");

#pragma GCC diagnostic pop
}

void InfypowerCANController::pm_query_count() {
    struct can_frame* can_frame;

    InfypowerCANCmd cmd("Read Module Count", InfypowerTxCANID(this->can_src_addr, this->can_dst_addr, cmd::read_info),
                        0x10, 0x10, 1);

    if (this->process_cmd(cmd))
        this->raise_incomplete_feedback(cmd);

    can_frame = cmd.get_rx_frame(0);
    this->pm_count = extract_uint16(&can_frame->data[6]);

    EVLOG_info << "The power module group consists of " << this->pm_count << " devices.";
}

void InfypowerCANController::pm_query_caps() {
    struct can_frame* can_frame;

    InfypowerCANCmd max_v("Read PM Caps DC Max Output Voltage",
                          InfypowerTxCANID(this->can_src_addr, this->can_dst_addr, cmd::read_info), 0x11, 0x30, 1);

    if (this->process_cmd(max_v))
        this->raise_incomplete_feedback(max_v);

    can_frame = max_v.get_rx_frame(0);
    this->caps.max_export_voltage_V = extract_uint32(&can_frame->data[4]) / 1000.0f;
    this->caps.max_import_voltage_V = this->caps.max_export_voltage_V;

    // ----

    InfypowerCANCmd min_v("Read PM Caps DC Min Output Voltage",
                          InfypowerTxCANID(this->can_src_addr, this->can_dst_addr, cmd::read_info), 0x11, 0x31, 1);

    if (this->process_cmd(min_v))
        this->raise_incomplete_feedback(min_v);

    can_frame = min_v.get_rx_frame(0);
    this->caps.min_export_voltage_V = extract_uint32(&can_frame->data[4]) / 1000.0f;
    this->caps.min_import_voltage_V = this->caps.min_export_voltage_V;

    // ----

    InfypowerCANCmd max_c("Read PM Caps DC Max Output Current",
                          InfypowerTxCANID(this->can_src_addr, this->can_dst_addr, cmd::read_info), 0x11, 0x32, 1);

    if (this->process_cmd(max_c))
        this->raise_incomplete_feedback(max_c);

    can_frame = max_c.get_rx_frame(0);
    this->caps.max_export_current_A = extract_uint32(&can_frame->data[4]) / 1000.0f;
    this->caps.max_import_current_A = this->caps.max_export_current_A;

    // ----

    InfypowerCANCmd max_p("Read PM Caps DC Max Output Power",
                          InfypowerTxCANID(this->can_src_addr, this->can_dst_addr, cmd::read_info), 0x11, 0x33, 1);

    if (this->process_cmd(max_p))
        this->raise_incomplete_feedback(max_p);

    can_frame = max_p.get_rx_frame(0);
    this->caps.max_export_power_W = extract_uint32(&can_frame->data[4]) / 1000.0f;
    this->caps.max_import_power_W = this->caps.max_export_power_W;

    // since we use the same values for both directions, we can summarize here
    EVLOG_info << "PM Capabilities: " << std::fixed << std::setprecision(1) << this->caps.min_export_voltage_V << "-"
               << std::fixed << std::setprecision(1) << this->caps.max_export_voltage_V << " V, " << std::fixed
               << std::setprecision(1) << this->caps.min_export_current_A << "-" << std::fixed << std::setprecision(1)
               << this->caps.max_export_current_A << " A, max " << std::fixed << std::setprecision(1)
               << this->caps.max_export_power_W << " W";
}

void InfypowerCANController::set_import_mode(bool enable_import) {
    unsigned char new_mode = enable_import ? 0xA1 : 0xA0;
    struct can_frame* can_frame;

    InfypowerCANCmd cmd("Set Working Mode", this->can_id_write_cfg, 0x21, 0x10, this->pm_count);

    can_frame = cmd.get_tx_frame();
    can_frame->data[7] = new_mode;

    if (this->process_cmd(cmd))
        this->raise_incomplete_feedback(cmd);

    for (unsigned int idx = 0; idx < this->pm_count; ++idx) {
        can_frame = cmd.get_rx_frame(idx);
        InfypowerCANID can_id(can_frame->can_id);

        if (can_id.get_error_code() != error_code::normal) {
            std::ostringstream os;
            os << can_id << " [" << can_frame << "]";
            throw std::runtime_error(os.str());
        }

        if (can_frame->data[7] != new_mode) {
            std::ostringstream os;
            os << "Feedback for requested mode differs for power module " << idx << ": expected 0x" << std::setw(2)
               << std::setfill('0') << std::hex << new_mode << ", got: 0x" << can_frame->data[7];
            EVLOG_error << os.str();
            throw std::runtime_error(os.str());
        }
    }
}

void InfypowerCANController::set_enable(bool enable) {
    unsigned char new_state = enable ? 0xA0 : 0xA1;
    struct can_frame* can_frame;
    InfypowerCANCmd cmd("Switch On/Off", this->can_id_write_cfg, 0x11, 0x10, this->pm_count);

    can_frame = cmd.get_tx_frame();
    can_frame->data[7] = new_state;

    if (this->process_cmd(cmd))
        this->raise_incomplete_feedback(cmd);

    for (unsigned int idx = 0; idx < this->pm_count; ++idx) {
        can_frame = cmd.get_rx_frame(idx);
        InfypowerCANID can_id(can_frame->can_id);

        if (can_id.get_error_code() != error_code::normal) {
            std::ostringstream os;
            os << can_id << " [" << can_frame << "]";
            throw std::runtime_error(os.str());
        }

        if (can_frame->data[7] != new_state) {
            std::ostringstream os;
            os << "Feedback for requested state differs for power module " << idx << ": expected 0x" << std::setw(2)
               << std::setfill('0') << std::hex << new_state << ", got: 0x" << can_frame->data[7];
            EVLOG_error << os.str();
            throw std::runtime_error(os.str());
        }
    }
}

void InfypowerCANController::raise_incomplete_feedback(const InfypowerCANCmd& cmd) {
    std::string errmsg = "Incomplete CAN feedback for ";
    errmsg += cmd;

    throw std::system_error(ETIMEDOUT, std::generic_category(), errmsg);
}

void InfypowerCANController::set_voltage_current(double voltage, double current) {
    struct can_frame* can_frame;
    int32_t val;

    this->requested_voltage = std::lround(voltage * 1000.0); // convert to mV
    this->requested_current = std::lround(current * 1000.0); // convert to mA

    // the group master replies with a single frame
    InfypowerCANCmd cmd_v("Set System DC Voltage", this->can_id_write_cfg, 0x10, 0x01, 1);

    // prepare payload
    can_frame = cmd_v.get_tx_frame();
    put_uint32(&can_frame->data[4], static_cast<uint32_t>(this->requested_voltage));

    if (this->process_cmd(cmd_v))
        this->raise_incomplete_feedback(cmd_v);

    can_frame = cmd_v.get_rx_frame(0);
    InfypowerCANID can_id_v(can_frame->can_id);

    if (can_id_v.get_error_code() != error_code::normal) {
        std::ostringstream os;
        os << can_id_v << " [" << can_frame << "]";
        throw std::runtime_error(os.str());
    }

    val = static_cast<int32_t>(extract_uint32(&can_frame->data[4]));
    if (val < this->requested_voltage - 1 || val > this->requested_voltage + 1)
        EVLOG_warning << "Feedback for requested voltage differs: requested " << this->requested_voltage
                      << " mV, feedback is " << val << " mV";

    // the group master replies with a single frame
    InfypowerCANCmd cmd_c("Set System DC Current", this->can_id_write_cfg, 0x10, 0x02, 1);

    // prepare payload
    can_frame = cmd_c.get_tx_frame();
    put_uint32(&can_frame->data[4], static_cast<uint32_t>(this->requested_current));

    if (this->process_cmd(cmd_c))
        this->raise_incomplete_feedback(cmd_c);

    can_frame = cmd_c.get_rx_frame(0);

    InfypowerCANID can_id_c(can_frame->can_id);

    if (can_id_c.get_error_code() != error_code::normal) {
        std::ostringstream os;
        os << can_id_c << " [" << can_frame << "]";
        throw std::runtime_error(os.str());
    }

    val = static_cast<int32_t>(extract_uint32(&can_frame->data[4]));
    if (val < this->requested_current - 1 || val > this->requested_current + 1)
        EVLOG_warning << "Feedback for requested current differs: requested " << this->requested_current
                      << " mA, feedback is " << val << " mA";
}

void InfypowerCANController::set_import_cutoff_voltage(double voltage) {
    struct can_frame* can_frame;

    this->requested_cutoff_voltage = std::lround(voltage * 1000.0); // convert to mV

    // the group master replies with a single frame
    InfypowerCANCmd cmd("Set Cut-Off DC Voltage", this->can_id_write_cfg, 0x11, 0x32, this->pm_count);

    // prepare payload
    can_frame = cmd.get_tx_frame();
    put_uint32(&can_frame->data[4], static_cast<uint32_t>(this->requested_cutoff_voltage));

    if (this->process_cmd(cmd))
        this->raise_incomplete_feedback(cmd);

    for (unsigned int idx = 0; idx < this->pm_count; ++idx) {
        int32_t val;

        can_frame = cmd.get_rx_frame(idx);
        InfypowerCANID can_id(can_frame->can_id);

        if (can_id.get_error_code() != error_code::normal) {
            std::ostringstream os;
            os << can_id << " [" << can_frame << "]";
            throw std::runtime_error(os.str());
        }

        val = static_cast<int32_t>(extract_uint32(&can_frame->data[4]));
        if (val < this->requested_cutoff_voltage - 1 || val > this->requested_cutoff_voltage + 1)
            EVLOG_warning << "Feedback for requested cut-off voltage differs: requested "
                          << this->requested_cutoff_voltage << " mV, feedback is " << val << " mV";
    }
}

bool InfypowerCANController::process_cmd(InfypowerCANCmd& cmd) {
    // Note: as soon as we register the cmd in the expected cmd list, the
    // worker thread might actually store feedback frames in cmd even that
    // we did not yet send out our CAN request.
    // This should usually not happen, but to ensure that we count the
    // received CAN frames correctly, we have to acquire the cmd's mutex already.
    // We need the mutex also so that we can wait on the cv later.
    std::unique_lock lock(cmd.mutex);

    // tell rx thread that we might receive feedback from now on
    this->register_expected_cmd(cmd);

    // send CAN frame
    std::string errmsg = "Could not send \"";
    errmsg += cmd;
    errmsg += "\"";
    this->push_to_can_raw(cmd.get_tx_frame(), errmsg);

    // sleep until we received all feedback, or timeout
    cmd.cv.wait_for(lock, this->request_timeout, [&] { return cmd.is_feedback_complete(); });

    // we are not expecting feedback anymore (or at least are not interested anymore)
    this->unregister_expected_cmd(cmd);

    // check whether we timed out
    if (!cmd.is_feedback_complete()) {
        EVLOG_error << "Incomplete feedback for " << cmd;
        return true;
    }

    return false;
}

void InfypowerCANController::can_bcm_rx_worker() {
    char bcm_frame[sizeof(struct bcm_msg_head) + sizeof(struct can_frame)];
    struct bcm_msg_head* bcm_msg_head = (struct bcm_msg_head*)bcm_frame;
    struct can_frame* can_frame = bcm_msg_head->frames;
    bool timeout_reported {false};

    EVLOG_debug << "CAN BCM Rx Thread started";

    while (!this->termination_requested) {
        int rv;

        // read blocks until new frame is available
        rv = read(this->can_bcm_fd, &bcm_frame, sizeof(bcm_frame));
        if (rv < 0)
            throw std::system_error(errno, std::generic_category(), "Couldn't read event from BCM");
        if (rv == 0)
            continue; // this should usually not happen
        // we should receive either a header plus frame, or only the header in case of RX_TIMEOUT
        if (rv != sizeof(struct bcm_msg_head) && rv != sizeof(bcm_frame))
            throw std::system_error(EBADMSG, std::generic_category(), "Short CAN BCM read");

        switch (bcm_msg_head->opcode) {
        case RX_CHANGED:
            if (bcm_msg_head->nframes != 1)
                continue; // this should usually not happen
            if (can_frame->len != 8)
                continue; // this should usually also not happen

            // since we only registered RX for a single CAN ID we trust BCM and do not check this further,
            // but can evaluate the data directly

            // since we have received a message, we can clear any error we raised before
            if (timeout_reported) {
                this->on_error(false, "CommunicationFault", "BCM RX_TIMEOUT", "");
                timeout_reported = false;
            }

            switch (can_frame->data[0]) {
            case 0x11:
                switch (can_frame->data[1]) {
                case 0x10:
                    this->on_error(can_frame->data[7] & (1 << 0), "VendorError", "Output Short Circuit",
                                   "Output Short Circuit");
                    this->on_error(can_frame->data[7] & (1 << 5), "VendorError", "Discharge Abnormal",
                                   "Discharge Abnormal");

                    this->on_error(can_frame->data[6] & (1 << 1), "VendorError", "Fault Alarm", "Fault Alarm");
                    this->on_error(can_frame->data[6] & (1 << 2), "VendorError", "Protection Alarm",
                                   "Protection Alarm");
                    this->on_error(can_frame->data[6] & (1 << 3), "VendorError", "Fan Fault Alarm", "Fan Fault Alarm");
                    this->on_error(can_frame->data[6] & (1 << 4), "OverTemperature", "", "OverTemperature");
                    this->on_error(can_frame->data[6] & (1 << 5), "OverVoltageDC", "", "OverVoltageDC");

                    this->on_error(can_frame->data[5] & (1 << 5), "UnderVoltageAC", "", "UnderVoltageAC");
                    this->on_error(can_frame->data[5] & (1 << 6), "OverVoltageAC", "", "OverVoltageAC");
                    break;
                }
                break;
            }

            break;

        case RX_TIMEOUT:
            if (!timeout_reported) {
                std::string errmsg = InfypowerCANID(bcm_msg_head->can_id);
                this->on_error(true, "CommunicationFault", "BCM RX_TIMEOUT", errmsg);
                timeout_reported = true;
            }
            break;

        default:
            throw std::system_error(EINVAL, std::generic_category(),
                                    "Unexpected BCM opcode received: " + std::to_string(bcm_msg_head->opcode));
        }
    }

    EVLOG_debug << "CAN BCM Rx Thread stopped";
}

void InfypowerCANController::push_to_can_bcm(const char* buf, size_t len, const std::string& err_msg) {
    size_t c = len;

    // loop until all data is written
    while (c) {
        int rv;

        rv = write(this->can_bcm_fd, &buf[len - c], c);
        if (rv < 0) {
            throw std::system_error(errno, std::generic_category(), err_msg);
        }

        c -= rv;
    }
}

void InfypowerCANController::push_to_can_raw(const struct can_frame* can_frame, const std::string& err_msg) {
    size_t c = sizeof(*can_frame);
    const char* buf = (const char*)can_frame;

    InfypowerCANID can_id(can_frame->can_id);
    EVLOG_debug << "push_to_can_raw: " << can_id << " [" << *can_frame << "]";

    // loop until all data is written
    while (c) {
        int rv;

        rv = write(this->can_raw_fd, &buf[sizeof(*can_frame) - c], c);
        if (rv < 0) {
            throw std::system_error(errno, std::generic_category(), err_msg);
        }

        c -= rv;
    }
}

void InfypowerCANController::setup_can_raw() {
    std::vector<InfypowerRxCANID> rx_can_id_list {
        // CAN ID for reading voltage/current errors and module status feedback (partly BCM related)
        InfypowerRxCANID(this->can_src_addr, cmd::read_info),

        // CAN ID used for feedback of writing settings
        InfypowerRxCANID(this->can_id_write_cfg),

        // we also want to listen for broadcasts
        InfypowerRxCANID(InfypowerCANID::BROADCAST_ADDR, cmd::read_info),
        InfypowerRxCANID(InfypowerCANID::BROADCAST_ADDR, cmd::write_cfg),
    };

    struct can_filter filter_list[rx_can_id_list.size()];

    // use traditional loop (so that we have a handy index) to
    // convert our high-level view list into filter structure
    for (size_t idx = 0; idx < rx_can_id_list.size(); ++idx) {
        filter_list[idx].can_id = rx_can_id_list[idx].can_id;
        filter_list[idx].can_mask = rx_can_id_list[idx].can_mask;
    }

    if (setsockopt(this->can_raw_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter_list, sizeof(filter_list)))
        throw std::system_error(errno, std::generic_category(), "Failed to setup CAN RAW filter on '" + device + "'");
}

void InfypowerCANController::can_raw_rx_worker() {
    struct can_frame raw_frame;

    EVLOG_debug << "CAN RAW Rx Thread started";

    while (!this->termination_requested) {
        int rv;

        // read blocks until new frame is available
        rv = read(this->can_raw_fd, &raw_frame, sizeof(raw_frame));
        if (rv < 0)
            throw std::system_error(errno, std::generic_category(), "Couldn't read CAN RAW frame");
        if (rv == 0)
            continue; // this should usually not happen
        // paranoid check :-)
        if (rv != sizeof(raw_frame))
            throw std::system_error(EBADMSG, std::generic_category(), "Short CAN RAW read");

        // all Infypower CAN frames have length of 8, so just ignore shorter frames
        if (raw_frame.len != 8)
            continue;

        // convert CAN ID to class for easier handling
        InfypowerCANID can_id(raw_frame.can_id);

        // filter out 'in_start_processing' (-> unclear whether needed to handle),
        // and 'cmd_invalid' (-> should not happen) -> just report
        switch (can_id.get_error_code()) {
        case error_code::in_start_processing:
            // FIXME not yet sure whether this should be handled
            EVLOG_debug << can_id;
            continue;

        case error_code::cmd_invalid:
            // this should not happen
            EVLOG_error << can_id;
            continue;

        default:
            break;
        }

        // log broadcasts for debugging
        if (can_id.get_dst_addr() == InfypowerCANID::BROADCAST_ADDR) {
            EVLOG_debug << can_id;
            continue;
        }

        std::scoped_lock lock(this->expected_cmds_mutex);

        // try to match the received CAN ID with one in our expectation list;
        // we also compare byte0 and byte1
        // FIXME: this must be reworked for REG module commands
        auto it = std::find_if(this->expected_cmds.begin(), this->expected_cmds.end(),
                               [&](const std::reference_wrapper<const InfypowerCANCmd>& ref) {
                                   if (ref.get() == can_id) {
                                       unsigned char exp_byte0 = ref.get().get_byte0();
                                       unsigned char exp_byte1 = ref.get().get_byte1();
                                       return (exp_byte0 == raw_frame.data[0]) && (exp_byte1 == raw_frame.data[1]);
                                   }
                                   return false;
                               });

        if (it != this->expected_cmds.end()) {
            if (it->get().store_feedback(can_id.get_src_addr(), &raw_frame)) {
                EVLOG_debug << "Stored feedback CAN frame for " << it->get();

                // signal to invoke possible waiters
                it->get().cv.notify_one();
            }
        } else {
            EVLOG_debug << "No match for this CAN ID in pending CMDs (size: " << this->expected_cmds.size()
                        << ") found: " << can_id;
        }
    }

    EVLOG_debug << "CAN RAW Rx Thread stopped";
}

void InfypowerCANController::register_expected_cmd(InfypowerCANCmd& cmd) {
    std::scoped_lock lock(this->expected_cmds_mutex);

    this->expected_cmds.push_back(cmd);
}

void InfypowerCANController::unregister_expected_cmd(InfypowerCANCmd& cmd) {
    std::scoped_lock lock(this->expected_cmds_mutex);

    auto it = std::find_if(this->expected_cmds.begin(), this->expected_cmds.end(),
                           [&cmd](const std::reference_wrapper<InfypowerCANCmd>& ref) { return &ref.get() == &cmd; });

    if (it != this->expected_cmds.end())
        this->expected_cmds.erase(it);
}
