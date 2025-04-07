// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include <ios>
#include <mutex>
#include <stdexcept>
#include <thread>
#include "power_supply_DCImpl.hpp"

namespace module {
namespace main {

void power_supply_DCImpl::init() {
}

void power_supply_DCImpl::ready() {
    std::scoped_lock lock(this->mutex);

    // register error handler
    this->mod->controller.on_error.connect(
        [this](bool error_present, const std::string& type, const std::string& sub_type, const std::string& errmsg) {
            if (error_present)
                this->raise_error("power_supply_DC/" + type, sub_type, errmsg);
            else
                this->clear_error("power_supply_DC/" + type, sub_type);
        });

    // use a working copy
    auto caps = this->mod->controller.caps;

    // apply possible overrides to the caps from configuration
    if (this->mod->config.override_max_current >= 0.0) {
        if (this->mod->config.override_max_current > caps.max_export_current_A) {
            EVLOG_warning << "NOT overriding capabilities for maximum export/import current with " << std::fixed
                          << std::setprecision(1) << caps.max_export_current_A << " A since the value is greater then "
                          << "the reported value by the power module itself with " << std::fixed << std::setprecision(1)
                          << caps.max_export_current_A << " A";
        } else {
            caps.max_export_current_A = this->mod->config.override_max_current;
            caps.max_import_current_A = this->mod->config.override_max_current;
            EVLOG_warning << "Applying override to capabilities for maximum export/import current: " << std::fixed
                          << std::setprecision(1) << caps.max_export_current_A << " A";
        }
    }
    if (this->mod->config.override_max_power >= 0.0) {
        if (this->mod->config.override_max_power > caps.max_export_power_W) {
            EVLOG_warning << "NOT overriding capabilities for maximum export/import power with " << std::fixed
                          << std::setprecision(1) << caps.max_export_power_W << " W since the value is greater then "
                          << "the reported value by the power module itself with " << std::fixed << std::setprecision(1)
                          << caps.max_export_power_W << " W";
        } else {
            caps.max_export_power_W = this->mod->config.override_max_power;
            caps.max_import_power_W = this->mod->config.override_max_power;
            EVLOG_warning << "Applying override to capabilities for maximum export/import power: " << std::fixed
                          << std::setprecision(1) << caps.max_export_power_W << " W";
        }
    }

    // capabilities must be published at least once
    this->publish_capabilities(caps);

    // publish our initial state to be on the safe side
    this->publish_mode(types::power_supply_DC::Mode::Off);

    // register (and thus enable) publishing of voltage and current values
    this->mod->controller.on_vc_update.connect([this](const float& voltage, const float& current) {
        types::power_supply_DC::VoltageCurrent vc;
        std::ostringstream dbgmsg;

        vc.voltage_V = voltage;
        vc.current_A = current;

        this->publish_voltage_current(vc);

        dbgmsg << std::fixed << std::setprecision(1) << "U: " << vc.voltage_V << " V, " << std::fixed
               << std::setprecision(1) << "I: " << vc.current_A << " A";

        if (this->last_vc_dbgmsg != dbgmsg.str()) {
            EVLOG_debug << dbgmsg.str();
            this->last_vc_dbgmsg = dbgmsg.str();
        }
    });
}

void power_supply_DCImpl::handle_setMode(types::power_supply_DC::Mode& mode,
                                         types::power_supply_DC::ChargingPhase& phase) {
    std::scoped_lock lock(this->mutex);

    EVLOG_info << "handle_setMode(" << mode << ", " << phase << ")";

    try {
        switch (mode) {
        case types::power_supply_DC::Mode::Export:
            this->mod->controller.set_import_mode(false);
            this->mod->controller.set_enable(true);
            break;
        case types::power_supply_DC::Mode::Import:
            this->mod->controller.set_import_mode(true);
            this->mod->controller.set_enable(true);
            break;
        default:
            this->mod->controller.set_enable(false);
        }

        // finally report the current mode
        this->publish_mode(mode);

    } catch (std::runtime_error& e) {
        this->raise_error("power_supply_DC/VendorError", "", e.what());
    }
}

void power_supply_DCImpl::handle_setExportVoltageCurrent(double& voltage, double& current) {
    std::scoped_lock lock(this->mutex);

    EVLOG_info << std::fixed << std::setprecision(1) << "handle_setExportVoltageCurrent(" << voltage << " V, "
               << current << " A)";
    try {
        this->mod->controller.set_voltage_current(voltage, current);

    } catch (std::runtime_error& e) {
        this->raise_error("power_supply_DC/VendorError", "", e.what());
    }
}

void power_supply_DCImpl::handle_setImportVoltageCurrent(double& voltage, double& current) {
    std::scoped_lock lock(this->mutex);

    EVLOG_info << std::fixed << std::setprecision(1) << "handle_setImportVoltageCurrent(" << voltage << " V, "
               << current << " A)";
    try {
        // since we do not have any other voltage from EVerest, we assume that
        // the voltage is carefully chosen and represents the cut-off voltage;
        // the user however might have configured an override in the configuration
        if (this->mod->config.override_cutoff_voltage >= 0.0)
            this->mod->controller.set_import_cutoff_voltage(this->mod->config.override_cutoff_voltage);
        else
            this->mod->controller.set_import_cutoff_voltage(voltage);

        // the voltage here is probably not important and ignored by the PM
        // but we re-use the function for setting the current
        this->mod->controller.set_voltage_current(voltage, current);

    } catch (std::runtime_error& e) {
        this->raise_error("power_supply_DC/VendorError", "", e.what());
    }
}

void power_supply_DCImpl::raise_comm_error(const std::string& errmsg) {
    bool old_value = this->comm_err_reported.exchange(true);

    // only if the old value (still) was false, then we can raise the error
    if (!old_value) {
        this->raise_error("power_supply_DC/CommunicationFault", "", errmsg);
        EVLOG_error << "CommunicationFault raised: " << errmsg;
    } else {
        EVLOG_debug << "CommunicationFault (suppressed): " << errmsg;
    }
}

void power_supply_DCImpl::clear_comm_error() {
    bool old_value = this->comm_err_reported.exchange(false);

    // only if the old value (still) was true, then we can clear the error
    if (old_value) {
        this->clear_error("power_supply_DC/CommunicationFault");
        EVLOG_info << "CommunicationFault cleared";
    } else {
        EVLOG_warning << "CommunicationFault clearing suppressed";
    }
}

void power_supply_DCImpl::raise_error(const std::string& type, const std::string& sub_type, const std::string& errmsg) {
    const auto err = this->error_factory->create_error(type, sub_type, errmsg);
    this->raise_error(err);
}

} // namespace main
} // namespace module
