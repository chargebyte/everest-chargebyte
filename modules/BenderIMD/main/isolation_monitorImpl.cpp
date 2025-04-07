// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "isolation_monitorImpl.hpp"
#include <iomanip>
#include <ios>

namespace module {
namespace main {

void isolation_monitorImpl::init() {
    // register callback to publish measured values
    this->mod->controller.on_update.connect([this](const float& resistance, const float& voltage) {
        types::isolation_monitor::IsolationMeasurement m;
        std::ostringstream dbgmsg;

        m.resistance_F_Ohm = resistance;
        m.voltage_V = voltage;

        this->publish_isolation_measurement(m);

        dbgmsg << std::fixed << std::setprecision(1) << "R: " << m.resistance_F_Ohm << " Ω, " << std::fixed
               << std::setprecision(1) << "V: " << m.voltage_V << " V";

        if (this->last_dbgmsg != dbgmsg.str()) {
            EVLOG_debug << dbgmsg.str();
            this->last_dbgmsg = dbgmsg.str();
        }
    });

    // register callback to publish self-test result
    this->mod->controller.on_selftest_finished.connect([this](bool success) {
        if (success)
            EVLOG_info << "IMD self-test succeeded.";
        else
            EVLOG_error << "IMD self-test failed.";

        this->publish_self_test_result(success);
    });
}

void isolation_monitorImpl::ready() {
}

void isolation_monitorImpl::handle_start() {
    EVLOG_info << "handle_start()";

    bool old_started = this->mod->started.exchange(true);

    if (old_started)
        return;

    this->mod->controller.start_measurement();
}

void isolation_monitorImpl::handle_stop() {
    EVLOG_info << "handle_stop()";

    bool old_started = this->mod->started.exchange(false);

    if (!old_started)
        return;

    this->mod->controller.stop_measurement();
}

void isolation_monitorImpl::handle_start_self_test(double& test_voltage_V) {
    EVLOG_info << "handle_start_self_test(" << std::fixed << std::setprecision(1) << test_voltage_V << " V)";

    this->mod->controller.trigger_self_test(test_voltage_V);
}

} // namespace main
} // namespace module
