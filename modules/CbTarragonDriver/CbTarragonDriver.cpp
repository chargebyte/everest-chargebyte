// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include "CbTarragonDriver.hpp"

namespace module {

void CbTarragonDriver::init() {
    invoke_init(*p_evse_board_support);
}

void CbTarragonDriver::ready() {
    invoke_ready(*p_evse_board_support);
}

} // namespace module
