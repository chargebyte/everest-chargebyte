// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once

#include <atomic>
#include <condition_variable>
#include <string>
#include <mutex>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sigslot/signal.hpp>
#include "InfypowerCANID.hpp"

///
/// A class for abstracting a Infypower data command sent via CAN with its query
/// and response. A command is here an action like setting the working mode, not
/// the command field within the CAN ID. Such an action is triggered by sending
/// a specific CAN frame and then it must be waited for at least one CAN frame
/// within a given timeout. It is also possible that multiple power modules
/// respond, in that case it is good to collect all feedback CAN frames.
///
class InfypowerCANCmd {

public:
    /// @brief Ctor
    /// @param name A simple string used for debugging purpose only
    /// @param tx_can_id The CAN ID used for sending the command
    /// @param byte0 Is the first byte in CAN data used in this command
    /// @param byte1 Is the second byte in CAN data used in this command
    /// @param rsp_count The expected count of CAN response frame, usually 1 or
    ///        the count of power modules in the current group. Use 0 for cyclic
    ///        message where infinite messages are expected.
    InfypowerCANCmd(const std::string& name, InfypowerTxCANID tx_can_id, unsigned char byte0, unsigned char byte1,
                    unsigned char rsp_count);

    /// @brief Helper to access the CAN frame when sending the command
    struct can_frame* get_tx_frame() {
        return &this->tx_frame;
    };

    /// @brief Helper to access the feedback CAN frames
    struct can_frame* get_rx_frame(unsigned int idx) {
        return &this->rx_frames[idx];
    };

    /// @brief Helper to access the CAN ID used for the query
    InfypowerTxCANID get_tx_can_id() const {
        return this->tx_frame.can_id;
    }

    /// @brief Helper to access the CAN ID expected for replies
    InfypowerRxCANID get_rx_can_id() const {
        return this->rx_can_id;
    }

    /// @brief Helper to read byte0 (used when matching this command)
    unsigned char get_byte0() const {
        return this->tx_frame.data[0];
    }

    /// @brief Helper to read byte1 (used when matching this command)
    unsigned char get_byte1() const {
        return this->tx_frame.data[1];
    }

    /// @brief Helper to allow comparing the received CAN ID with this command
    bool operator==(const InfypowerCANID& other_can_id) const {
        return (other_can_id.can_id & this->rx_can_id.can_mask) == (this->rx_can_id.can_id & this->rx_can_id.can_mask);
    }

    /// @brief Helper for representing an instance as string
    operator std::string() const {
        return this->name;
    }

    /// @brief Helper to store a single feedback frame for later evaluation (acquires mutex)
    /// @return True if the feedback frame was actually stored, false otherwise (e.g. no feedback expected).
    bool store_feedback(unsigned int src_addr, struct can_frame* rcv_frame);

    /// @brief Returns whether the count of received feedback is equal to the
    ///        expected one. Must be called with mutex held.
    bool is_feedback_complete() const {
        return this->rx_frames_expected == this->rx_frames_used;
    }

    /// @brief This signal is used as callback when a frame was received.
    ///        At the moment, it is only used for cyclic messages (i.e. rsp_count == 0).
    sigslot::signal<struct can_frame*> on_received;

    /// @brief Mutex to serialize member access to e.g. rx_frames.
    std::mutex mutex;

    /// @brief Used for waiting for feedback and signaling.
    std::condition_variable cv;

    /// @brief Feeds a string representation of the given instance into an output stream.
    /// @return A reference to the output stream operated on.
    friend std::ostream& operator<<(std::ostream& os, const InfypowerCANCmd& cmd);

private:
    /// @brief According to documentation, at max 60 power modules can be grouped,
    ///        but documentation also states that 61 addresses can be configured,
    ///        so let's assume the worst case.
    static constexpr unsigned char MAX_PM_MODULE = 61;

    /// @brief Remember our name
    std::string name;

    /// @brief The CAN ID we expect for responses
    InfypowerRxCANID rx_can_id;

    /// @brief Holds the constructed CAN frame which is sent out
    struct can_frame tx_frame;

    /// @brief Remember how many responses we expect to this command
    unsigned int rx_frames_expected;

    /// @brief Helper to collect all response from different modules;
    ///        if there is only one frame expected, it is always stored at offset 0
    ///        in all other cases, it is stored at index = source address.
    struct can_frame rx_frames[MAX_PM_MODULE + 1];

    /// @brief Helper to count how many elements in rx_frames are already used.
    unsigned int rx_frames_used {0};
};

std::ostream& operator<<(std::ostream& os, const struct can_frame& frame);
