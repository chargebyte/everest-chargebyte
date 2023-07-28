// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include "CbTarragonDriver.hpp"

namespace module {

void CbTarragonDriver::init() {
    invoke_init(*p_low_level);
}

void CbTarragonDriver::ready() {
    invoke_ready(*p_low_level);
}

} // namespace module
