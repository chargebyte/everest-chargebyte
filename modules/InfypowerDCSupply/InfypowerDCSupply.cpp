// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include "InfypowerDCSupply.hpp"
#include "configuration.h"

namespace module {

void InfypowerDCSupply::init() {
    EVLOG_info << MODULE_DESCRIPTION << " (version: " << MODULE_VERSION << ")";

    try {
        this->controller.init(this->config.device, this->config.bitrate, this->config.can_source_address,
                              this->config.can_destination_address, this->config.dc_module_type);
    } catch (std::system_error& e) {
        EVLOG_error << e.what();
        return;
    }

    invoke_init(*p_main);
}

void InfypowerDCSupply::ready() {
    invoke_ready(*p_main);
}

} // namespace module
