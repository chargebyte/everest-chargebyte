// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include "EvseHardwareManager.hpp"

namespace module {

void EvseHardwareManager::init() {
    invoke_init(*p_board_support);
}

void EvseHardwareManager::ready() {
    invoke_ready(*p_board_support);
}

} // namespace module
