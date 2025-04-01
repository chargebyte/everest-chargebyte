// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest

#include <cstring>
#include <iostream>
#include <iomanip>
#include <mutex>
#include <stdexcept>
#include <string>
#include <linux/can.h>
#include "InfypowerCANCmd.hpp"

InfypowerCANCmd::InfypowerCANCmd(const std::string& name, InfypowerTxCANID tx_can_id, unsigned char byte0,
                                 unsigned char byte1, unsigned char rsp_count) :
    name(name),
    //  we cannot use the InfypowerTxCANID specific ctor since we want narrow can_mask
    rx_can_id(tx_can_id.get_src_addr(), tx_can_id.get_cmd()),
    rx_frames_expected(rsp_count) {

    memset(&this->tx_frame, 0, sizeof(this->tx_frame));
    this->tx_frame.can_id = tx_can_id;
    this->tx_frame.len = 8;
    this->tx_frame.data[0] = byte0;
    this->tx_frame.data[1] = byte1;

    memset(&this->rx_frames, 0, sizeof(this->rx_frames));
}

std::ostream& operator<<(std::ostream& os, const InfypowerCANCmd& cmd) {
    os << cmd.name << " (exp: " << cmd.rx_frames_expected << ",rcv: " << cmd.rx_frames_used << ")";
    return os;
}

bool InfypowerCANCmd::store_feedback(unsigned int src_addr, struct can_frame* rcv_frame) {
    std::scoped_lock lock(this->mutex);
    struct can_frame* dst;

    if (this->rx_frames_expected == 0) {
        // in this use case we don't store the frame but evaluate the received
        // content in the callback function
        this->on_received(rcv_frame);
        return false;
    }

    if (this->rx_frames_expected <= 1) {
        dst = &this->rx_frames[0];
    } else {
        if (src_addr > this->MAX_PM_MODULE)
            throw std::runtime_error("Unexpected source address received: " + std::to_string(src_addr));

        dst = &this->rx_frames[src_addr];
    }

    memcpy(dst, rcv_frame, sizeof(*dst));

    this->rx_frames_used++;

    return true;
}

std::ostream& operator<<(std::ostream& os, const struct can_frame& frame) {
    for (unsigned int idx = 0; idx < frame.len; ++idx) {
        if (idx > 0)
            os << " ";
        os << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(frame.data[idx]);
    }
    return os;
}
