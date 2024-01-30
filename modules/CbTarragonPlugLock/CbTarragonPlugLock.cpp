// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include "CbTarragonPlugLock.hpp"

namespace module {

void CbTarragonPlugLock::init() {
    invoke_init(*p_connector_lock);
}

void CbTarragonPlugLock::ready() {
    invoke_ready(*p_connector_lock);
}

} // namespace module
