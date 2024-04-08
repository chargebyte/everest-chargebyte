// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include "CbTarragonPlugLock.hpp"
#include "configuration.h"

namespace module {

void CbTarragonPlugLock::init() {
    invoke_init(*p_connector_lock);

    EVLOG_info << MODULE_DESCRIPTION << " (version: " << MODULE_VERSION << ")";
}

void CbTarragonPlugLock::ready() {
    invoke_ready(*p_connector_lock);
}

} // namespace module
