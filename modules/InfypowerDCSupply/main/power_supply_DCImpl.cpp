// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "power_supply_DCImpl.hpp"

namespace module {
namespace main {

void power_supply_DCImpl::init() {
}

void power_supply_DCImpl::ready() {
}

void power_supply_DCImpl::handle_setMode(types::power_supply_DC::Mode& mode,
                                         types::power_supply_DC::ChargingPhase& phase) {
    // your code for cmd setMode goes here
}

void power_supply_DCImpl::handle_setExportVoltageCurrent(double& voltage, double& current) {
    // your code for cmd setExportVoltageCurrent goes here
}

void power_supply_DCImpl::handle_setImportVoltageCurrent(double& voltage, double& current) {
    // your code for cmd setImportVoltageCurrent goes here
}

} // namespace main
} // namespace module
