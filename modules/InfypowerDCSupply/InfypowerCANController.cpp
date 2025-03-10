// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include "InfypowerCANController.hpp"

#include <cerrno>
#include <cinttypes>
#include <cstdint>
#include <iosfwd>
#include <iostream>
#include <string>
#include <stdexcept>
#include <system_error>
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
    // struct Capabilities {
    //     bool bidirectional; ///< 'true: support bidirectional energy flow, false: support only Export mode (output)'
    //     float current_regulation_tolerance_A; ///< Absolute magnitude of the regulation tolerance in Ampere
    //     float peak_current_ripple_A; ///< Peak-to-peak magnitude of the current ripple in Ampere
    //     float max_export_voltage_V; ///< Maximum voltage that the power supply can output in Volt
    //     float min_export_voltage_V; ///< Minimum voltage that the power supply can output in Volt
    //     float max_export_current_A; ///< Maximum current that the power supply can output in Ampere
    //     float min_export_current_A; ///< Minimum current limit that the power supply can set in Ampere
    //     float max_export_power_W; ///< Maximum export power that the power supply can output in Watt
    //     std::optional<float> max_import_voltage_V; ///< Maximum voltage that the power supply supports to import
    //     energy in Volt std::optional<float> min_import_voltage_V; ///< Minimum voltage that the power supply requires
    //     to import energy in Volt std::optional<float> max_import_current_A; ///< Maximum current that the power
    //     supply can output in Ampere std::optional<float> min_import_current_A; ///< Minimum current limit that the
    //     power supply can set in Ampere std::optional<float> max_import_power_W; ///< Maximum import power that the
    //     power supply can sink in Watt std::optional<float> conversion_efficiency_import; ///< Typical import
    //     efficiency used for energy management std::optional<float> conversion_efficiency_export; ///< Typical export
    //     efficiency used for energy management
    // };
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
    this->query_pm_count();
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

    // we use this CAN ID to trigger sending periodic responses
    InfypowerTxCANID tx_can_id(this->can_src_addr, this->can_dst_addr, cmd::read_info);

    // thus we must then listen for this (better: similar, aka reply) CAN ID: derive it from our sending one
    InfypowerRxCANID rx_can_id(tx_can_id);

    struct {
        struct bcm_msg_head bcm_msg_head;
        struct can_frame can_frame[3];
    } __attribute__((packed)) bcm_rx_setup = {
        .bcm_msg_head =
            {
                .opcode = RX_SETUP,
                .flags = SETTIMER | RX_ANNOUNCE_RESUME,
                .count = 0,
                .ival1 =
                    {
                        .tv_sec = 0,
                        .tv_usec = 210000, // expect frames in intervals of 200 ms plus 10 ms safety margin
                    },
                .ival2 =
                    {
                        .tv_sec = 0,
                        .tv_usec = 250000, // limit updates to once every 250ms (see EVerest's recommendation)
                    },
                .can_id = rx_can_id,
                .nframes = 3,
            },
        .can_frame =
            {
                {
                    .can_id = rx_can_id,
                    .len = 8,
                    .__pad = 0,
                    .__res0 = 0,
                    .len8_dlc = 0,
                    .data = {0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // MUX mask
                },
                {
                    .can_id = rx_can_id,
                    .len = 8,
                    .__pad = 0,
                    .__res0 = 0,
                    .len8_dlc = 0,
                    .data = {0x00, 0x01, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff}, // mask for System DC side voltage
                },
                {
                    .can_id = rx_can_id,
                    .len = 8,
                    .__pad = 0,
                    .__res0 = 0,
                    .len8_dlc = 0,
                    .data = {0x00, 0x02, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff}, // mask for System DC side current
                },
            },
    };

    // push the setup request to CAN BCM
    this->push_to_can_bcm((const char*)&bcm_rx_setup, sizeof(bcm_rx_setup),
                          "Couldn't register response frames with BCM");

    struct {
        struct bcm_msg_head bcm_msg_head;
        struct can_frame can_frame[2];
    } __attribute__((packed)) bcm_tx_setup = {
        .bcm_msg_head =
            {
                .opcode = TX_SETUP,
                .flags = SETTIMER | STARTTIMER | TX_CP_CAN_ID,
                .count = 0,
                .ival1 =
                    {
                        .tv_sec = 0,
                        .tv_usec = 0,
                    },
                .ival2 =
                    {
                        .tv_sec = 0,
                        .tv_usec =
                            100000, // send at intervals of 100 ms alternating 2 frames -> effective cycle is 200ms
                    },
                .can_id = tx_can_id,
                .nframes = 2,
            },
        .can_frame =
            {
                {
                    .can_id = tx_can_id,
                    .len = 8,
                    .__pad = 0,
                    .__res0 = 0,
                    .len8_dlc = 0,
                    .data = {0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // System DC side voltage
                },
                {
                    .can_id = tx_can_id,
                    .len = 8,
                    .__pad = 0,
                    .__res0 = 0,
                    .len8_dlc = 0,
                    .data = {0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // System DC side current
                },
            },
    };

    // push the setup request to CAN BCM
    this->push_to_can_bcm((const char*)&bcm_tx_setup, sizeof(bcm_tx_setup),
                          "Couldn't register request frames with BCM");

#pragma GCC diagnostic pop
}

void InfypowerCANController::query_pm_count() {
    struct can_frame* can_frame;

    InfypowerCANCmd cmd("Read Module Count", InfypowerTxCANID(this->can_src_addr, this->can_dst_addr, cmd::read_info),
                        0x10, 0x10, 1);

    if (this->process_cmd(cmd))
        this->raise_incomplete_feedback(cmd);

    can_frame = cmd.get_rx_frame(0);
    this->pm_count = extract_uint16(&can_frame->data[6]);

    EVLOG_info << "The power module group consists of " << this->pm_count << " devices.";
}

void InfypowerCANController::set_import_mode(bool enable_import) {
    unsigned char new_mode = enable_import ? 0xA1 : 0xA2;
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
    unsigned char new_state = enable ? 0xA1 : 0xA2;
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

void InfypowerCANController::raise_incomplete_feedback(InfypowerCANCmd& cmd) {
    std::string errmsg = "Incomplete CAN feedback for ";
    errmsg += cmd;

    throw std::system_error(ETIMEDOUT, std::generic_category(), errmsg);
}

void InfypowerCANController::set_voltage_current(double voltage, double current) {
    struct can_frame* can_frame;
    float f;

    this->requested_voltage = voltage * 1000; // convert to mV
    this->requested_current = current * 1000; // convert to mA

    // the group master replies with a single frame
    InfypowerCANCmd cmd_v("Set System DC Voltage", this->can_id_write_cfg, 0x10, 0x01, 1);

    // prepare payload
    can_frame = cmd_v.get_tx_frame();
    put_float(&can_frame->data[4], this->requested_voltage);

    if (this->process_cmd(cmd_v))
        this->raise_incomplete_feedback(cmd_v);

    can_frame = cmd_v.get_rx_frame(0);
    InfypowerCANID can_id_v(can_frame->can_id);

    if (can_id_v.get_error_code() != error_code::normal) {
        std::ostringstream os;
        os << can_id_v << " [" << can_frame << "]";
        throw std::runtime_error(os.str());
    }

    f = extract_float(&can_frame->data[4]);
    if (f != this->requested_voltage)
        EVLOG_warning << "Feedback for requested voltage differs: requested " << std::fixed << std::setprecision(0)
                      << this->requested_voltage << " mV, feedback is " << f << " mV";

    // the group master replies with a single frame
    InfypowerCANCmd cmd_c("Set System DC Current", this->can_id_write_cfg, 0x10, 0x02, 1);

    // prepare payload
    can_frame = cmd_c.get_tx_frame();
    put_float(&can_frame->data[4], this->requested_current);

    if (this->process_cmd(cmd_c))
        this->raise_incomplete_feedback(cmd_c);

    can_frame = cmd_c.get_rx_frame(0);

    InfypowerCANID can_id_c(can_frame->can_id);

    if (can_id_c.get_error_code() != error_code::normal) {
        std::ostringstream os;
        os << can_id_c << " [" << can_frame << "]";
        throw std::runtime_error(os.str());
    }

    f = extract_float(&can_frame->data[4]);
    if (f != this->requested_current)
        EVLOG_warning << "Feedback for requested current differs: requested " << std::fixed << std::setprecision(0)
                      << this->requested_current << " mA, feedback is " << f << " mA";
}

bool InfypowerCANController::process_cmd(InfypowerCANCmd& cmd) {
    // tell rx thread that we might receive feedback from now on
    this->register_expected_cmd(cmd);

    // send CAN frame
    std::string errmsg = "Could not send \"";
    errmsg += cmd;
    errmsg += "\"";
    this->push_to_can_raw(cmd.get_tx_frame(), errmsg);

    // acquire lock before waiting
    std::unique_lock lock(cmd.mutex);

    // sleep until we received all feedback, or timeout
    cmd.cv.wait_for(lock, this->request_timeout, [&] { return cmd.is_feedback_complete(); });

    // we not expecting feedback anymore (or at least are not interested anymore)
    this->unregister_expected_cmd(cmd);

    // check whether we timed out
    if (!cmd.is_feedback_complete()) {
        EVLOG_error << "Incomplete feedback for " << cmd;
        return true;
    }

    return false;
}

void InfypowerCANController::can_bcm_rx_worker() {
    struct {
        struct bcm_msg_head bcm_msg_head;
        struct can_frame can_frame;
    } __attribute__((packed)) bcm_frame;
    bool timeout_reported {false};
    float voltage {0.0f};
    float current {0.0f};

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
        if (rv != sizeof(bcm_frame.bcm_msg_head) && rv != sizeof(bcm_frame))
            throw std::system_error(EBADMSG, std::generic_category(), "Short CAN BCM read");

        switch (bcm_frame.bcm_msg_head.opcode) {
        case RX_CHANGED:
            if (bcm_frame.bcm_msg_head.nframes != 1)
                continue; // this should usually not happen
            if (bcm_frame.can_frame.len != 8)
                continue; // this should usually also not happen

            // since we only registered RX for a single CAN ID we trust BCM and do not check this further,
            // but can evaluate the data directly

            // since we have received a message, we can clear any error we raised before
            if (timeout_reported) {
                this->on_error(false, "CommunicationFault", "BCM RX_TIMEOUT", "");
                timeout_reported = false;
            }

            // Note: EVerest expects voltage and current bundled but we receive in two different messages
            // so just publish when receiving the current and "cache"/"delay" the voltage value until then

            switch (bcm_frame.can_frame.data[0]) {
            case 0x10:
                switch (bcm_frame.can_frame.data[1]) {
                case 0x01:
                    voltage = extract_float(&bcm_frame.can_frame.data[4]) / 1000.0f;
                    break;
                case 0x02:
                    current = extract_float(&bcm_frame.can_frame.data[4]) / 1000.0f;

                    this->on_vc_update(voltage, current);
                    break;
                }
                break;
            }

            break;

        case RX_TIMEOUT:
            if (!timeout_reported) {
                std::string errmsg = InfypowerCANID(bcm_frame.bcm_msg_head.can_id);
                this->on_error(true, "CommunicationFault", "BCM RX_TIMEOUT", errmsg);
                timeout_reported = true;
            }
            break;

        default:
            throw std::system_error(EINVAL, std::generic_category(),
                                    "Unexpected BCM opcode received: " + std::to_string(bcm_frame.bcm_msg_head.opcode));
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

    // loop until all data is written
    while (c) {
        int rv;

        rv = write(this->can_raw_fd, &buf[sizeof(can_frame) - c], c);
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

        // try to match the received CAN ID with one in our expectation list
        auto it = std::find_if(
            this->expected_cmds.begin(), this->expected_cmds.end(),
            [&can_id](const std::reference_wrapper<const InfypowerCANCmd>& ref) { return ref.get() == can_id; });

        if (it != this->expected_cmds.end()) {
            // we found a match, let's compare byte0 and byte1
            struct can_frame* can_frame = it->get().get_tx_frame();
            bool byteX_mismatch = false;

            for (unsigned int b_idx = 0; b_idx < 2; ++b_idx) {
                if (can_frame->data[b_idx] != raw_frame.data[b_idx]) {
                    EVLOG_debug << it->get() << ": ignoring due to byte" << b_idx << " mismatch: expected 0x"
                                << std::setw(2) << std::setfill('0') << std::hex << can_frame->data[b_idx]
                                << ", got: 0x" << raw_frame.data[b_idx] << " [" << raw_frame << "]";
                    byteX_mismatch = true;
                    break;
                }
            }
            if (byteX_mismatch)
                continue;

            it->get().store_feedback(can_id.get_src_addr(), &raw_frame);
            EVLOG_debug << "Stored feedback CAN frame for " << it->get();

            // signal to invoke possible waiters
            it->get().cv.notify_one();
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
