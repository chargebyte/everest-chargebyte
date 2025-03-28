// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once

#include <iosfwd>
#include <iostream>
#include <sstream>
#include <string>
#include <linux/can.h>

// clang-format off
/// CAN identifier:
///
/// +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+---+---+---+---+---+---+---+---+---+---+
/// | 28 | 27 | 26 | 25 | 24 | 23 | 22 | 21 | 20 | 19 | 18 | 17 | 16 | 15 | 14 | 13 | 12 | 11 | 10 | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
/// +----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+---+---+---+---+---+---+---+---+---+---+
/// |  Error Code  |     Device No     |          Command No         |         Destination Address         |         Source Address        |
/// +--------------+-------------------+-----------------------------+-------------------------------------+-------------------------------+
///
// clang-format on

// not all values are specified
enum class error_code : unsigned char
{
    normal = 0x00,
    cmd_invalid = 0x02,
    data_invalid = 0x03,
    in_start_processing = 0x07,
};

std::ostream& operator<<(std::ostream& os, const error_code& ec);

// there are only two directions: controller to single module vs. controller to module group
enum class device_no : unsigned char
{
    single_module = 0x0A,
    module_group = 0x0B,
};

std::ostream& operator<<(std::ostream& os, const device_no& dn);

// Note: not all commands are understood by each power module series!
// For example, the BEC/BEG series only use 0x23 and 0x24.
enum class cmd : unsigned char
{
    read_system_info_01 = 0x01,
    read_system_info_02 = 0x02,
    read_module_info_03 = 0x03,
    read_module_info_04 = 0x04,
    read_module_input_voltage_info = 0x06,
    read_module_version_info = 0x07,
    read_module_info_0A = 0x0A,
    read_module_info_0B = 0x0B,
    read_module_info_0E = 0x0E,
    write_module_cfg_0F = 0x0F,
    write_module_cfg_13 = 0x13,
    write_blink = 0x14,
    write_group_number = 0x16,
    write_on_off = 0x1A,
    write_system_output = 0x1B,
    write_module_output = 0x1C,
    write_module_addr_alloc_mode = 0x1F,
    read_info = 0x23,
    write_cfg = 0x24,
};

std::ostream& operator<<(std::ostream& os, const cmd& cmd);

///
/// A class for abstracting the Infypower's CAN IDs
///
class InfypowerCANID {

public:
    /// @brief Default/dump ctor
    InfypowerCANID() {};

    /// @brief Ctor from elements with usually used parameters
    InfypowerCANID(unsigned char src_addr, unsigned char dst_addr, cmd cmd);

    /// @brief Ctor from elements with all possible parameters
    InfypowerCANID(unsigned char src_addr, unsigned char dst_addr, cmd cmd, device_no device_no, error_code error_code);

    /// @brief Ctor from CAN ID
    InfypowerCANID(canid_t can_id);

    /// @brief Returns the source address
    unsigned char get_src_addr() const;

    /// @brief Returns the destination address
    unsigned char get_dst_addr() const;

    /// @brief Returns the CMD
    cmd get_cmd() const;

    /// @brief Returns the device no
    device_no get_device_no() const;

    /// @brief Returns the error code
    error_code get_error_code() const;

    /// @brief The broadcast address can be used as source and destination address
    static constexpr unsigned char BROADCAST_ADDR = 0x3F;

    /// @brief Helper that an instance can directly assigned to the low-level struct fields
    operator canid_t() const {
        return this->can_id;
    }

    /// @brief Helper for representing an instance as string
    operator std::string() const {
        std::ostringstream s;
        this->dump(s);
        return s.str();
    }

    /// @brief Comparison operator
    bool operator==(const InfypowerCANID& other) const {
        return this->can_id == other.can_id;
    }

    /// @brief The resulting CAN ID
    canid_t can_id {0};

    /// @brief The CAN mask when used in as filter
    canid_t can_mask {CAN_EFF_MASK};

    /// @brief Feeds a string representation of the given InfypowerCANID
    ///        instance into an output stream.
    /// @return A reference to the output stream operated on.
    friend std::ostream& operator<<(std::ostream& os, const InfypowerCANID& can_id);

protected:
    /// @brief Internal helper to update the CAN ID member
    void update_can_id(unsigned char src_addr, unsigned char dst_addr, cmd cmd, device_no device_no,
                       error_code error_code);

    /// @brief Helper to extract a field from the CAN ID
    unsigned char get_field(const unsigned char shift, const unsigned char len) const;

    /// @brief Helper to set a field in the CAN ID
    canid_t set_field(const unsigned char val, const unsigned char shift, const unsigned char len) const;

    /// @brief Helper to feed a string representation into an output stream.
    /// @param os Output stream reference to operate on.
    /// @return A reference to the output stream operated on.
    virtual std::ostream& dump(std::ostream& os) const;

    /// @brief The various shifts and lengths of the individual CAN ID fields.
    static constexpr unsigned char ERROR_CODE_LEN = 3;
    static constexpr unsigned char ERROR_CODE_SHIFT = 26;
    static constexpr unsigned char DEVICE_NO_LEN = 4;
    static constexpr unsigned char DEVICE_NO_SHIFT = 22;
    static constexpr unsigned char CMD_LEN = 6;
    static constexpr unsigned char CMD_SHIFT = 16;
    static constexpr unsigned char DST_ADDR_LEN = 8;
    static constexpr unsigned char DST_ADDR_SHIFT = 8;
    static constexpr unsigned char SRC_ADDR_LEN = 8;
    static constexpr unsigned char SRC_ADDR_SHIFT = 0;
};

///
/// A class for abstracting the Infypower's CAN ID, especially for transmission
/// i.e. the can_mask field is left unchanged.
///
class InfypowerTxCANID : public InfypowerCANID {

public:
    /// @brief Default/dump ctor
    InfypowerTxCANID() {};

    /// @brief Ctor from CAN ID
    InfypowerTxCANID(canid_t can_id) : InfypowerCANID(can_id) {};

    /// @brief Ctor from elements with usually used parameters
    InfypowerTxCANID(unsigned char src_addr, unsigned char dst_addr, cmd cmd);

    /// @brief Feeds a string representation of the given InfypowerCANID
    ///        instance into an output stream.
    /// @return A reference to the output stream operated on.
    friend std::ostream& operator<<(std::ostream& os, const InfypowerTxCANID& can_id);
};

///
/// A class for abstracting the Infypower's CAN ID, especially for reception.
/// There are two main use-cases:
/// - for BCM RX_SETUP structure construction: here we usually can derive the
///   CAN ID from the corresponding CAN ID used for TX_SETUP
///   but need to reverse src and dst
/// - for CAN RAW sockets filter construction: here the source address and
///   the error code fields can be masked out, so that we don't need to specify
///   each combination of source address and/or error code
///
class InfypowerRxCANID : public InfypowerCANID {

public:
    /// @brief Ctor from InfypowerTxCANID, here the src_addr and dst_addr are swapped
    ///        automatically.
    InfypowerRxCANID(const InfypowerTxCANID& c);

    /// @brief Ctor from elements with usually used parameters (dst_addr is used as passed,
    ///        src_addr and error code are masked out)
    InfypowerRxCANID(unsigned char dst_addr, cmd cmd);

    /// @brief Feeds a string representation of the given InfypowerCANID
    ///        instance into an output stream.
    /// @return A reference to the output stream operated on.
    friend std::ostream& operator<<(std::ostream& os, const InfypowerRxCANID& can_id);
};
