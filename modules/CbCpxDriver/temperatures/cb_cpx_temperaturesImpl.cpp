// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include <chrono>
#include <functional>
#include <string>
#include <thread>
#include <vector>
#include "cb_cpx_temperaturesImpl.hpp"

using namespace std::chrono_literals;

namespace module {
namespace temperatures {

void cb_cpx_temperaturesImpl::init() {
}

void cb_cpx_temperaturesImpl::ready() {
    this->publish_thread = std::thread([&]() {
        unsigned int supported_channels = this->mod->controller->get_temperature_channels();
        const std::vector<std::reference_wrapper<const std::string>> ident_config{
            std::ref(this->mod->config.pt1000_1_identification),
            std::ref(this->mod->config.pt1000_2_identification),
            std::ref(this->mod->config.pt1000_3_identification),
            std::ref(this->mod->config.pt1000_4_identification)
        };

        while (!this->mod->termination_requested) {
            std::vector<types::temperature::Temperature> v;

            // we sleep here to not overfill MQTT buffer
            std::this_thread::sleep_for(1s);

            if (this->mod->termination_requested)
                break;

            // this check whether we received temperature data at least once
            if (!this->mod->controller->temperature_data_is_valid)
                continue;

            for (unsigned int i = 1; i <= supported_channels; ++i) {
                types::temperature::Temperature t;

                // skip this channel if not enabled at all
                if (!this->mod->controller->is_temperature_enabled(i))
                    continue;

                if ((i - 1) < ident_config.size()) {
                    t.identification = ident_config[i - 1].get();
                } else {
                    t.identification = "Channel " + std::to_string(i);
                }

                t.temperature = this->mod->controller->get_temperature(i);

                if (this->mod->controller->is_pt_charging_stopped(i)) {
                    if (!this->charging_abort_cause_reported[i]) {
                        EVLOG_warning << t.identification.value() << " caused charging abort: " << std::fixed
                                      << t.temperature << " °C";
                        this->charging_abort_cause_reported[i] = true;
                    }
                } else {
                    // we can just reset the flag here since we don't want to
                    // notify the user explicitly because the port was reset completely
                    if (this->charging_abort_cause_reported[i]) {
                        EVLOG_warning << t.identification.value() << " charging abort flag reset";
                        this->charging_abort_cause_reported[i] = false;
                    }
                }

                if (this->mod->controller->is_pt_selftest_failed(i)) {
                    if (!this->selftest_failed_reported[i]) {
                        EVLOG_error << "Self-test for " << t.identification.value() << " failed.";
                        this->selftest_failed_reported[i] = true;
                    }
                    // unsure whether the data is (still) valid at all, so don't forward it anymore
                    continue;
                } else {
                    // we can just reset the flag here since we don't want to
                    // notify the user explicitly because the port was reset completely
                    if (this->selftest_failed_reported[i]) {
                        EVLOG_warning << t.identification.value() << " selftest failed flag reset";
                        this->selftest_failed_reported[i] = false;
                    }
                }

                v.push_back(t);
            }

            this->publish_temperatures(v);
        }
    });
}

} // namespace temperatures
} // namespace module
