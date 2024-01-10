#pragma once
#include <chrono>
#include <memory>
#include <string>
#include <gpiod.hpp>

/// @brief Class representing the 3 possible contactor feedback types.
enum class CbContactorFeedbackType {
    NONE,
    NORMALLY_OPEN,
    NORMALLY_CLOSED
};

class CbTarragonRelay {

public:
    /// @brief Default constructor
    CbTarragonRelay(void);

    /// @brief Constructor
    /// @param contactor_relay Name of the relay and its feedback as labeled on hardware.
    /// @param actuator_gpio_line_name The name of the GPIO line which switches the relay on/off.
    /// @param feedback_type Defines the logic behind the feedback (no = normally open, nc = normally close, none = no feedback).
    /// @param feedback_gpio_line_name The name of the GPIO line to which the feedback/sense signal of relay 1 is connected to.
    CbTarragonRelay(const std::string &relay_name,
                    const std::string &actuator_gpio_line_name,
                    const std::string &feedback_type,
                    const std::string &feedback_gpio_line_name);

    /// @brief Set the timestamp of the last contactor state change from closed to open
    void set_last_contactor_open_ts(std::chrono::time_point<std::chrono::steady_clock> timestamp);

    /// @brief Set the flag `delay_contactor_close`.
    void set_delay_contactor_close(bool value);

    /// @brief Check if the contactor could be closed immediately i.e. switching the relay on, or
    ///        if this operation should be delayed to prevent internal relay wear.
    /// @return `true` if it can be closed, `false` if delayed
    bool can_close_contactor(void);

    // @brief Return the value of contactor_close_interval.
    std::chrono::seconds get_contactor_close_interval(void);

    /// @brief Switch the relay on/off.
    /// @param new_state_on The target state (0 = off, 1 = on).
    void set_actuator_state(bool new_state_on);

    /// @brief Read the actual GPIO state of the actuator.
    bool get_actuator_state(void);

    /// @brief Function to map the string feedback type to CbContactorFeedbackType.
    /// @param feedback_type String representing the feedback type (no = normally open, nc = normally close, none = no feedback).
    /// @return Return NONE as default, NORMALLY_OPEN, NORMALLY_CLOSE.
    CbContactorFeedbackType get_feedback_type(std::string feedback_type);

    /// @brief Read the actual state of the GPIO of the contactor feedback.
    bool get_feedback_state(void);

    /// @brief Wait for duration of 'timeout' for a new event to happen on the GPIO.
    /// @param timeout The duration to wait for an event to happen.
    /// @return Return '1' in case there are new events and '0' if otherwise
    bool wait_for_feedback(const std::chrono::nanoseconds &timeout) const;

    /// @brief Read events in a buffer. This is a blocking function and therefore it should
    ///        be used only if there are new events to read i.e., in combination with the function wait_for_feedback.
    /// @return Return an edge event (FALLING_EDGE or RISING_EDGE).
    gpiod::edge_event::event_type read_feedback_event(void);

    /// @brief Map edge events (FALLING_EDGE, RISING_EDGE)to contactor states.
    /// @return Return '0' if opened, '1' if closed.
    bool verify_feedback_event(gpiod::edge_event::event_type event);

private:
    /// @brief Name of the relay and its feedback as labeled on hardware.
    std::string relay_name;

    /// @brief Define the logic behind the feedback (no = normally open, nc = normally close, none = no feedback).
    CbContactorFeedbackType feedback_type;

    /// @brief Buffer to store the new edge event
    gpiod::edge_event_buffer feedback_event_buffer;

    /// @brief The GPIO handle of the used actuator (RELAY_1_ENABLE or RELAY_2_ENABLE).
    std::unique_ptr<gpiod::line_request> actuator;

    /// @brief The GPIO handle of the used feedback (RELAY_1_SENSE or RELAY_2_SENSE).
    std::unique_ptr<gpiod::line_request> feedback;

    /// @brief The timestamp which records the moment when the contactor's state transitioned
    ///        from closed to open.
    std::chrono::time_point<std::chrono::steady_clock> last_contactor_open_ts;

    /// @brief The time in seconds which represents a minimum duration between contactor
    ///        closings to prevent internal relay wear. The worst considered contactor type
    //         can withstand a maximum of 6 cycles (open and close actions) per
    ///        minute i.e., 10 seconds between 2 contactor closings.
    std::chrono::seconds contactor_close_interval;

    /// @brief A flag that is set to `true` when it is needed to delay closing of the contactor
    ///        to prevent the internal relay wear.
    bool delay_contactor_close;
};
