// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include "CbTarragonDriver.hpp"
#include "configuration.h"

namespace module {

void CbTarragonDriver::init() {
    invoke_init(*p_evse_board_support);
    invoke_init(*p_ac_rcd);


    EVLOG_info << MODULE_DESCRIPTION << " (version: " << MODULE_VERSION << ")";
}

void CbTarragonDriver::ready() {
    invoke_ready(*p_evse_board_support);
    invoke_ready(*p_ac_rcd);
}

} // namespace module
