// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#include "CbCANLogMark.hpp"
#include "configuration.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <system_error>

#include <can_netlink.h>
#include <libsocketcan.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace module {

namespace {

constexpr int RECEIVE_POLL_TIMEOUT_MS = 250;
constexpr canid_t SFF_FILTER_MASK = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;

std::string payload_to_string(const can_frame& frame) {
    const auto* begin = reinterpret_cast<const char*>(frame.data);
    const auto* end = begin + CAN_MAX_DLEN;
    const auto* string_end = std::find(begin, end, '\0');
    return {begin, string_end};
}

} // namespace

CbCANLogMark::~CbCANLogMark() {
    this->stop();
}

void CbCANLogMark::init() {
    EVLOG_info << MODULE_DESCRIPTION << " (version: " << MODULE_VERSION << ")";

    try {
        this->validate_config();
        this->configure_and_open_can();
        this->receive_thread = std::thread(&CbCANLogMark::receive_worker, this);
    } catch (const std::exception& e) {
        EVLOG_error << "Failed to initialize CbCANLogMark: " << e.what();
        this->stop();
    }
}

void CbCANLogMark::ready() {
}

void CbCANLogMark::validate_config() const {
    const std::array<int, 4> configured_ids {
        this->config.error_can_id,
        this->config.warning_can_id,
        this->config.info_can_id,
        this->config.debug_can_id,
    };

    auto sorted_ids = configured_ids;
    std::sort(sorted_ids.begin(), sorted_ids.end());
    if (std::adjacent_find(sorted_ids.begin(), sorted_ids.end()) != sorted_ids.end()) {
        throw std::invalid_argument("Configured CAN IDs must be unique");
    }
}

void CbCANLogMark::configure_and_open_can() {
    can_bittiming bit_timing {};
    int state;

    if (can_get_state(this->config.device.c_str(), &state) != 0) {
        throw std::system_error(errno, std::generic_category(), "Failed to open '" + this->config.device + "'");
    }

    if (can_get_bittiming(this->config.device.c_str(), &bit_timing) != 0) {
        throw std::system_error(errno, std::generic_category(),
                                "Failed to retrieve current bitrate of '" + this->config.device + "'");
    }

    if (bit_timing.bitrate != static_cast<__u32>(this->config.bitrate)) {
        if (state != CAN_STATE_STOPPED) {
            if (can_do_stop(this->config.device.c_str()) != 0) {
                throw std::system_error(errno, std::generic_category(),
                                        "Could not stop '" + this->config.device + "' before re-configuring");
            }
            state = CAN_STATE_STOPPED;
        }

        if (can_set_bitrate(this->config.device.c_str(), this->config.bitrate) != 0) {
            throw std::system_error(errno, std::generic_category(),
                                    "Could not re-configure bitrate on '" + this->config.device + "'");
        }
    }

    if (state == CAN_STATE_STOPPED && can_do_start(this->config.device.c_str()) != 0) {
        throw std::system_error(errno, std::generic_category(), "Could not start '" + this->config.device + "'");
    }

    this->can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (this->can_fd == -1) {
        throw std::system_error(errno, std::generic_category(), "socket(PF_CAN, CAN_RAW) failed");
    }

    const std::array<can_filter, 4> filters {{
        {static_cast<canid_t>(this->config.error_can_id), SFF_FILTER_MASK},
        {static_cast<canid_t>(this->config.warning_can_id), SFF_FILTER_MASK},
        {static_cast<canid_t>(this->config.info_can_id), SFF_FILTER_MASK},
        {static_cast<canid_t>(this->config.debug_can_id), SFF_FILTER_MASK},
    }};

    if (setsockopt(this->can_fd, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), sizeof(filters)) != 0) {
        throw std::system_error(errno, std::generic_category(),
                                "Failed to setup CAN RAW filters on '" + this->config.device + "'");
    }

    const unsigned int interface_index = if_nametoindex(this->config.device.c_str());
    if (interface_index == 0) {
        throw std::system_error(errno, std::generic_category(),
                                "Could not determine interface number of '" + this->config.device + "'");
    }

    sockaddr_can address {};
    address.can_family = AF_CAN;
    address.can_ifindex = static_cast<int>(interface_index);
    if (bind(this->can_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        throw std::system_error(errno, std::generic_category(),
                                "Could not bind CAN RAW socket on '" + this->config.device + "'");
    }
}

void CbCANLogMark::receive_worker() {
    EVLOG_debug << "CAN RAW receive thread started";

    pollfd descriptor {
        this->can_fd,
        POLLIN,
        0,
    };

    while (!this->termination_requested) {
        descriptor.revents = 0;
        const int poll_result = poll(&descriptor, 1, RECEIVE_POLL_TIMEOUT_MS);
        if (poll_result == 0) {
            continue;
        }
        if (poll_result < 0) {
            if (errno == EINTR) {
                continue;
            }
            EVLOG_error << "Could not poll CAN RAW socket: " << std::strerror(errno);
            break;
        }
        if ((descriptor.revents & POLLIN) == 0) {
            EVLOG_error << "CAN RAW socket reported an unexpected event";
            break;
        }

        can_frame frame {};
        const ssize_t received = read(this->can_fd, &frame, sizeof(frame));
        if (received < 0) {
            if (errno == EINTR || errno == EAGAIN) {
                continue;
            }
            EVLOG_error << "Could not read CAN RAW frame: " << std::strerror(errno);
            break;
        }
        if (received != static_cast<ssize_t>(sizeof(frame)) || frame.len != CAN_MAX_DLEN ||
            (frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0) {
            continue;
        }

        const canid_t can_id = frame.can_id & CAN_SFF_MASK;
        const std::string message = payload_to_string(frame);
        if (can_id == static_cast<canid_t>(this->config.error_can_id)) {
            EVLOG_error << message;
        } else if (can_id == static_cast<canid_t>(this->config.warning_can_id)) {
            EVLOG_warning << message;
        } else if (can_id == static_cast<canid_t>(this->config.info_can_id)) {
            EVLOG_info << message;
        } else if (can_id == static_cast<canid_t>(this->config.debug_can_id)) {
            EVLOG_debug << message;
        }
    }

    EVLOG_debug << "CAN RAW receive thread stopped";
}

void CbCANLogMark::stop() {
    this->termination_requested = true;

    if (this->receive_thread.joinable()) {
        this->receive_thread.join();
    }

    if (this->can_fd != -1) {
        close(this->can_fd);
        this->can_fd = -1;
    }
}

} // namespace module
