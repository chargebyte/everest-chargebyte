// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include "CbSystem.hpp"

namespace module {

void CbSystem::init() {
    invoke_init(*p_main);
}

void CbSystem::ready() {
    invoke_ready(*p_main);
}

} // namespace module
