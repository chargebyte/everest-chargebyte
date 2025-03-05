// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest

#include <iostream>
#include <iomanip>
#include <linux/can.h>
#include "InfypowerCANID.hpp"

std::ostream& operator<<(std::ostream& os, const error_code& ec) {
    switch (ec) {
    case error_code::normal:
        os << "Normal";
        break;
    case error_code::cmd_invalid:
        os << "Command Invalid";
        break;
    case error_code::data_invalid:
        os << "Data Invalid";
        break;
    case error_code::in_start_processing:
        os << "In Start Processing";
        break;
    default:
        os << "Unknown";
    }
    // we show with 2 digits since the datasheet also uses 2
    os << "(0x" << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(ec) << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const device_no& dn) {
    switch (dn) {
    case device_no::single_module:
        os << "Controller \u2194 Single Module";
        break;
    case device_no::module_group:
        os << "Controller \u2194 Module Group";
        break;
    default:
        os << "Unknown";
    }
    os << "(0x" << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(dn) << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const cmd& cmd) {
    os << "0x" << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(cmd);
    return os;
}

InfypowerCANID::InfypowerCANID(unsigned char src_addr, unsigned char dst_addr, cmd cmd) {
    this->update_can_id(src_addr, dst_addr, cmd, device_no::module_group, error_code::normal);
}

InfypowerCANID::InfypowerCANID(unsigned char src_addr, unsigned char dst_addr, cmd cmd, device_no device_no,
                               error_code error_code) {
    this->update_can_id(src_addr, dst_addr, cmd, device_no, error_code);
}

InfypowerCANID::InfypowerCANID(canid_t can_id) {
    this->can_id = can_id;
}

unsigned char InfypowerCANID::get_field(const unsigned char shift, const unsigned char len) const {
    return (this->can_id >> shift) & ((1 << len) - 1);
}

canid_t InfypowerCANID::set_field(const unsigned char val, const unsigned char shift, const unsigned char len) const {
    return (val & ((1 << len) - 1)) << shift;
}

unsigned char InfypowerCANID::get_src_addr() const {
    return this->get_field(SRC_ADDR_SHIFT, SRC_ADDR_LEN);
}

unsigned char InfypowerCANID::get_dst_addr() const {
    return this->get_field(DST_ADDR_SHIFT, DST_ADDR_LEN);
}

cmd InfypowerCANID::get_cmd() const {
    return static_cast<cmd>(this->get_field(CMD_SHIFT, CMD_LEN));
}

device_no InfypowerCANID::get_device_no() const {
    return static_cast<device_no>(this->get_field(DEVICE_NO_SHIFT, DEVICE_NO_LEN));
}

error_code InfypowerCANID::get_error_code() const {
    return static_cast<error_code>(this->get_field(ERROR_CODE_SHIFT, ERROR_CODE_LEN));
}

void InfypowerCANID::update_can_id(unsigned char src_addr, unsigned char dst_addr, cmd cmd, device_no device_no,
                                   error_code error_code) {

    this->can_id = this->set_field(src_addr, SRC_ADDR_SHIFT, SRC_ADDR_LEN) |
                   this->set_field(dst_addr, DST_ADDR_SHIFT, DST_ADDR_LEN) |
                   this->set_field(static_cast<unsigned char>(cmd), CMD_SHIFT, CMD_LEN) |
                   this->set_field(static_cast<unsigned char>(device_no), DEVICE_NO_SHIFT, DEVICE_NO_LEN) |
                   this->set_field(static_cast<unsigned char>(error_code), ERROR_CODE_SHIFT, ERROR_CODE_LEN);
    this->can_id |= CAN_EFF_FLAG;
}

std::ostream& InfypowerCANID::dump(std::ostream& os) const {
    os << "CAN ID 0x" << std::setw(8) << std::setfill('0') << std::hex << this->can_id
       << " (EC:" << this->get_error_code() << ",DN:" << this->get_device_no() << ",CMD:" << this->get_cmd();

    if (this->get_dst_addr() == InfypowerCANID::BROADCAST_ADDR)
        os << ",DST:Broadcast";
    else
        os << ",DST:0x" << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(this->get_dst_addr());

    if (this->get_src_addr() == InfypowerCANID::BROADCAST_ADDR)
        os << ",SRC:Broadcast";
    else
        os << ",SRC:0x" << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(this->get_src_addr());

    os << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const InfypowerCANID& can_id) {
    return can_id.dump(os);
}

InfypowerTxCANID::InfypowerTxCANID(unsigned char src_addr, unsigned char dst_addr, cmd cmd) :
    InfypowerCANID(src_addr, dst_addr, cmd) {
}

std::ostream& operator<<(std::ostream& os, const InfypowerTxCANID& can_id) {
    return can_id.dump(os);
}

InfypowerRxCANID::InfypowerRxCANID(const InfypowerTxCANID& c) :
    // swapped dst and src, but can_mask is kept with default preset because
    // here the use-case is to construct a BCM RX_SETUP structure
    InfypowerCANID(c.get_dst_addr(), c.get_src_addr(), c.get_cmd(), c.get_device_no(), error_code::normal) {
}

InfypowerRxCANID::InfypowerRxCANID(unsigned char dst_addr, cmd cmd) :
    // we use the broadcast address here as source but this is not important since we mask it out
    InfypowerCANID(InfypowerCANID::BROADCAST_ADDR, dst_addr, cmd) {

    // only set the bits which are important during rx;
    // the other field are kind of 'dont care' so that we can use this as filter
    this->can_mask = this->set_field((1 << DST_ADDR_LEN) - 1, DST_ADDR_SHIFT, DST_ADDR_LEN) |
                     this->set_field((1 << CMD_LEN) - 1, CMD_SHIFT, CMD_LEN) |
                     this->set_field((1 << DEVICE_NO_LEN) - 1, DEVICE_NO_SHIFT, DEVICE_NO_LEN);
}

std::ostream& operator<<(std::ostream& os, const InfypowerRxCANID& can_id) {
    return can_id.dump(os);
}
