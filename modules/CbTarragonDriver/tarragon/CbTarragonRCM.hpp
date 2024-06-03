// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once
#include <string>
#include <gpiod.hpp>
#include <memory>

class CbTarragonRCM {

public:
    /// @brief Default constructor
    CbTarragonRCM(void);

    /// @brief Constructor
    /// @param rcm_fault_gpio_line_name The name of the GPIO line to which the RCM fault is connected to.
    /// @param rcm_fault_active_low The GPIO polarity of RCM fault pin (active low = true, active high = false)
    CbTarragonRCM(const std::string& rcm_fault_gpio_line_name, const bool& rcm_fault_active_low);

    /// @brief Check whether the RCM is tripped.
    /// @return True if the RCM is tripped, false otherwise.
    bool is_rcm_tripped() const;

    /// @brief Wait for an RCM event to occur.
    /// @param timeout The maximum time to wait for the event in nanoseconds.
    void wait_for_rcm_event(const std::chrono::nanoseconds& timeout);

private:
    /// @brief The GPIO handle of the used RCM fault pin.
    std::unique_ptr<gpiod::line_request> rcm_fault;
};
