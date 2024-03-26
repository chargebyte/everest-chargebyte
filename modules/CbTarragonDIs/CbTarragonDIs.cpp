// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include "CbTarragonDIs.hpp"
#include "tarragon/CbTarragonDIPWM.hpp"
#include "configuration.h"

namespace module {

void CbTarragonDIs::init() {
    invoke_init(*p_empty);

    EVLOG_info << PROJECT_DESCRIPTION << " (version: " << PROJECT_VERSION << ")";

    tarragon_di_pwm =
        new CbTarragonDIPWM(this->config.pwm_device, this->config.pwmchannel, this->config.threshold_voltage);
}

void CbTarragonDIs::ready() {
    invoke_ready(*p_empty);
}

} // namespace module
