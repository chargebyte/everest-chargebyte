// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include "CbTarragonDIs.hpp"

namespace module {

void CbTarragonDIs::init() {
    invoke_init(*p_empty);
}

void CbTarragonDIs::ready() {
    invoke_ready(*p_empty);
}

} // namespace module
