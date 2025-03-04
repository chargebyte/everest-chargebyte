// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include "InfypowerDCSupply.hpp"

namespace module {

void InfypowerDCSupply::init() {
    invoke_init(*p_power_supply_DC);
}

void InfypowerDCSupply::ready() {
    invoke_ready(*p_power_supply_DC);
}

} // namespace module
