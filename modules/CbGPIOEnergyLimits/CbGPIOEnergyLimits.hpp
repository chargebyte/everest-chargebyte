// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH, Pionix GmbH and Contributors to EVerest
#ifndef CB_GPIOENERGY_LIMITS_HPP
#define CB_GPIOENERGY_LIMITS_HPP

//
// AUTO GENERATED - MARKED REGIONS WILL BE KEPT
// template version 2
//

#include "ld-ev.hpp"

// headers for required interface implementations
#include <generated/interfaces/external_energy_limits/Interface.hpp>

// ev@4bf81b14-a215-475c-a1d3-0a484ae48918:v1
// insert your custom include headers here
#include <algorithm>
#include <fstream>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <optional>
#include <vector>
#include <gpiod.hpp>

// first: current limit, optional: phase count limit
using LimitItem = std::tuple<float, std::optional<int>>;

using GpioLineRequest = std::unique_ptr<gpiod::line_request>;

// gpio line name, active_low
using GpioLine = std::tuple<std::string, bool>;

// ev@4bf81b14-a215-475c-a1d3-0a484ae48918:v1

namespace module {

struct Conf {};

class CbGPIOEnergyLimits : public Everest::ModuleBase {
public:
    CbGPIOEnergyLimits() = delete;
    CbGPIOEnergyLimits(const ModuleInfo& info, std::vector<std::unique_ptr<external_energy_limitsIntf>> r_energy_sink,
                       Conf& config) :
        ModuleBase(info), r_energy_sink(std::move(r_energy_sink)), config(config) {};

    const std::vector<std::unique_ptr<external_energy_limitsIntf>> r_energy_sink;
    const Conf& config;

    // ev@1fce4c5e-0ab8-41bb-90f7-14277703d2ac:v1
    // insert your public definitions here
    // ev@1fce4c5e-0ab8-41bb-90f7-14277703d2ac:v1

protected:
    // ev@4714b2ab-a24f-4b95-ab81-36439e1478de:v1
    // insert your protected definitions here
    // ev@4714b2ab-a24f-4b95-ab81-36439e1478de:v1

private:
    friend class LdEverest;
    void init();
    void ready();

    // ev@211cfdbe-f69a-4cd6-a4ec-f8aaa3d1b6c8:v1
    // insert your private definitions here

    // the list of DT compatible strings of the board we are running on
    std::vector<std::string> dt_compatibles;

    // helper which initializes the DT compat list
    void init_dt_compatibles_list();

    // helper to check whether a given value is in the DT compat list
    bool is_dt_compatible(const std::string& value);

    // helper to check whether one of the given values is in the DT compat list
    bool is_dt_compatible(std::initializer_list<std::string_view> values);

    // holds the acquired GPIO handles
    std::vector<GpioLineRequest> gpios;

    // helper to acquire a single GPIO line
    GpioLineRequest acquire_gpio(const GpioLine& line);

    // read the status of all GPIO lines and combine into an integer which can
    // be used as offset for the following vector
    std::size_t get_gpios();

    // list of limits (max current and optional a max phase count);
    // the list should have an entry for each possible item which use can encode
    std::vector<LimitItem> limits;

    // helper to publish (new) limit to all linked energy nodes
    void apply_new_limit(const float current_limit_A, std::optional<int> max_phase_count);

    // ev@211cfdbe-f69a-4cd6-a4ec-f8aaa3d1b6c8:v1
};

// ev@087e516b-124c-48df-94fb-109508c7cda9:v1
// insert other definitions here
// ev@087e516b-124c-48df-94fb-109508c7cda9:v1

} // namespace module

#endif // CB_GPIOENERGY_LIMITS_HPP
