// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include <chrono>
#include <thread>
#include "cb_chargesom_temperaturesImpl.hpp"

using namespace std::chrono_literals;

namespace module {
namespace temperatures {

void cb_chargesom_temperaturesImpl::init() {
}

void cb_chargesom_temperaturesImpl::ready() {
    this->publish_thread = std::thread([&]() {
        unsigned int supported_channels = this->mod->controller.get_temperature_channels();

        while (!this->mod->termination_requested) {
            std::vector<types::temperature::Temperature> v;

            for (unsigned int i = 0; i < supported_channels; ++i) {
                types::temperature::Temperature t;

                if (!this->mod->controller.is_temperature_enabled(i) or !this->mod->controller.is_temperature_valid(i))
                    continue;

                switch (i) {
                case 0:
                    t.identification = this->mod->config.pt1000_1_identification;
                    break;
                case 1:
                    t.identification = this->mod->config.pt1000_2_identification;
                    break;
                case 2:
                    t.identification = this->mod->config.pt1000_3_identification;
                    break;
                case 3:
                    t.identification = this->mod->config.pt1000_4_identification;
                    break;
                default:
                    t.identification = "Channel " + std::to_string(i + 1);
                }

                t.temperature = this->mod->controller.get_temperature(i);

                v.push_back(t);
            }

            this->publish_temperatures(v);

            std::this_thread::sleep_for(1s);
        }
    });
}

} // namespace temperatures
} // namespace module
