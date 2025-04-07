// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include "BenderIMD.hpp"
#include "configuration.h"

namespace module {

void BenderIMD::init() {
    invoke_init(*p_main);

    EVLOG_info << MODULE_DESCRIPTION << " (version: " << MODULE_VERSION << ")";

    this->controller.mb_read_holding_registers = [&](unsigned int addr, unsigned int count) {
        return this->r_serial_comm_hub->call_modbus_read_holding_registers(this->config.modbus_address, addr, count);
    };

    this->controller.mb_write_single_register = [&](unsigned int addr, int data) {
        return this->r_serial_comm_hub->call_modbus_write_single_register(this->config.modbus_address, addr, data);
    };
}

void BenderIMD::ready() {
    invoke_ready(*p_main);

    // query the initial and static device properties
    this->controller.init();
}

} // namespace module
