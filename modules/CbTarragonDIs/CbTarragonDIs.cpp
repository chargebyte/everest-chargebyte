// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#include <everest/logging.hpp>

#include "CbTarragonDIs.hpp"
#include "configuration.h"
#include "tarragon/CbTarragonDIPWM.hpp"

namespace module {

void CbTarragonDIs::init() {
    invoke_init(*p_empty);

    EVLOG_info << MODULE_DESCRIPTION << " (version: " << MODULE_VERSION << ")";

    this->tarragon_di_pwm = std::make_unique<CbTarragonDIPWM>(this->config.pwm_device, this->config.pwmchannel,
                                                              this->config.threshold_voltage);
}

void CbTarragonDIs::ready() {
    invoke_ready(*p_empty);
}

} // namespace module
