#pragma once
#include <string>
#include <gpiod.hpp>
#include <memory>

class CbTarragonRCM {

public:
    /// @brief Default constructor
    CbTarragonRCM(void);

    /// @brief Constructor
    /// @param contactor_relay Name of the relay and its feedback as labeled on hardware.
    /// @param actuator_gpio_line_name The name of the GPIO line which switches the relay on/off.
    /// @param feedback_type Defines the logic behind the feedback (no = normally open, nc = normally close, none = no feedback).
    /// @param feedback_gpio_line_name The name of the GPIO line to which the feedback/sense signal of relay 1 is connected to.
    CbTarragonRCM(const std::string &rcm_fault_gpio_line_name,
                  const std::string &rcm_fault_active_low);

    /// @brief Check whether the RCM is tripped.
    /// @return True if the RCM is tripped, false otherwise.
    bool is_rcm_tripped() const;

    /// @brief Wait for an RCM event to occur.
    /// @param timeout The maximum time to wait for the event in nanoseconds.
    void wait_for_rcm_event(const std::chrono::nanoseconds &timeout);

private:
    /// @brief The GPIO handle of the used RCM fault pin.
    std::unique_ptr<gpiod::line_request> rcm_fault;
};
