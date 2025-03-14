// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include <ios>
#include <mutex>
#include <stdexcept>
#include <thread>
#include "power_supply_DCImpl.hpp"

namespace module {
namespace power_supply_DC {

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
    if (this->mod->config.override_max_current >= 0.0f) {
        caps.max_export_current_A = this->mod->config.override_max_current;
        caps.max_import_current_A = this->mod->config.override_max_current;
        EVLOG_warning << "Applying override to capabilities for maximum export/import current: " << std::fixed
                      << std::setprecision(1) << caps.max_export_current_A << " A";
    }
    if (this->mod->config.override_max_power >= 0.0f) {
        caps.max_export_power_W = this->mod->config.override_max_power;
        caps.max_import_power_W = this->mod->config.override_max_power;
        EVLOG_warning << "Applying override to capabilities for maximum export/import power: " << std::fixed
                      << std::setprecision(1) << caps.max_export_power_W << " W";
    }

    // capabilities must be published at least once
    this->publish_capabilities(caps);

    // FIXME do we need to publish our initial state?
    this->publish_mode(types::power_supply_DC::Mode::Off);

    // register (and this enable) publishing of voltage and current values
    this->mod->controller.on_vc_update.connect([this](float& voltage, float& current) {
        types::power_supply_DC::VoltageCurrent vc;

        vc.voltage_V = voltage;
        vc.current_A = current;

        this->publish_voltage_current(vc);
        EVLOG_debug << std::fixed << std::setprecision(1) << "U: " << vc.voltage_V << " V, " << std::fixed
                    << std::setprecision(1) << "I: " << vc.current_A << " A";
    });
}

void power_supply_DCImpl::handle_setMode(types::power_supply_DC::Mode& mode,
                                         types::power_supply_DC::ChargingPhase& phase) {
    std::scoped_lock lock(this->mutex);

    EVLOG_info << "handle_setMode(" << mode << ", " << phase << ")";

    try {
        switch (mode) {
        case types::power_supply_DC::Mode::Export:
            // this->mod->controller.set_import_mode(false);
            this->mod->controller.set_enable(true);
            break;
        case types::power_supply_DC::Mode::Import:
            // this->mod->controller.set_import_mode(true);
            this->mod->controller.set_enable(true);
            break;
        default:
            this->mod->controller.set_enable(false);
        }

        // remember the current charging phase
        this->charging_phase = phase;

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
        auto new_current = current;

        if (this->charging_phase == types::power_supply_DC::ChargingPhase::Charging) {
            new_current = -1.0 * 2.0;
        }

        this->mod->controller.set_voltage_current(voltage, new_current);

    } catch (std::runtime_error& e) {
        this->raise_error("power_supply_DC/VendorError", "", e.what());
    }
}

void power_supply_DCImpl::handle_setImportVoltageCurrent(double& voltage, double& current) {
    std::scoped_lock lock(this->mutex);

    EVLOG_debug << std::fixed << std::setprecision(1) << "handle_setImportVoltageCurrent(" << voltage << " V, "
                << current << " A)";
    try {
        this->mod->controller.set_voltage_current(voltage, -1.0 * std::abs(current));

    } catch (std::runtime_error& e) {
        this->raise_error("power_supply_DC/VendorError", "", e.what());
    }
}

void power_supply_DCImpl::raise_comm_error(const std::string& errmsg) {
    bool old_value = this->comm_err_reported.exchange(true);

    // only if the old value (still) was false, then we can raise the error
    if (!old_value) {
        const auto err = this->error_factory->create_error("power_supply_DC/CommunicationFault", "", errmsg);
        this->raise_error(err);
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

} // namespace power_supply_DC
} // namespace module
