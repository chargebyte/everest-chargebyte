// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include "InfypowerDCSupply.hpp"

namespace module {

void InfypowerDCSupply::init() {
    invoke_init(*p_main);
}

void InfypowerDCSupply::ready() {
    invoke_ready(*p_main);
}

} // namespace module
