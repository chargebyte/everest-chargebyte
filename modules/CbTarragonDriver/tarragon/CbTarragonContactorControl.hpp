// SPDX-License-Identifier: Apache-2.0
// Copyright chargebyte GmbH and Contributors to EVerest
#pragma once
#include <chrono>
#include <optional>
#include <string>
#include <iostream>
#include "CbTarragonRelay.hpp"

/// @brief Class representing the 2 possible contactor states.
enum class ContactorState {
    CONTACTOR_UNKNOWN,
    CONTACTOR_OPEN,
    CONTACTOR_CLOSED
};

/// @brief Writes the string representation of the given ContactorState to the given
///        output stream os.
/// @return an output stream with the state written to
std::ostream& operator<<(std::ostream& os, const ContactorState& state);

/// @brief Class representing the different state types.
enum class StateType {
    ACTUAL_STATE,
    TARGET_STATE
};

///
/// This class abstracts the control of the two relays on the Tarragon platform. Each relay controls
/// the opening or closing of a contactor in a charging station, and the feedback representing the state of
/// the contactor can be read and analyzed through the board. This class simplifies the relay control,
/// allowing for handling both single and three-phase operations.
/// For phase-count switching setups, only the so called 'serial' wiring setup is supported.
/// This means, that the primary contactor (controlled via relay 1) either switches all 3 phases or
/// at least the first one, and the secondary contactor switches phase 2 and 3. But the important aspect
/// here is that the secondary contactor is first switch on, and switched off last, while the primary
/// contactor is switch on last, and switched off first. The sequence is required to present a consistent
/// "theses phase are available" view to the car - in other words, the phases do not arrive with a
/// time lag at the car.
/// Note on the implementation: in case phase-count switching is not enabled, we interally handle this
/// as single phase mode (because in this case, the real-world number of phases does not matter).
///
class CbTarragonContactorControl {

public:
    /// @brief Default constructor.
    CbTarragonContactorControl(void);

    /// @brief Constructor.
    /// @param relay_1_name The name of the first relay and its feedback as labeled on hardware.
    /// @param relay_1_actuator_gpio_line_name The name of the GPIO line which switches the first relay on/off.
    /// @param relay_1_feedback_gpio_line_name The name of the GPIO line to which the feedback/sense signal of relay 1
    /// is connected to.
    /// @param relay_1_feedback_gpio_deboucne_us The debounce period which should be configured on the GPIO line in
    /// microseconds. Pass zero to skip the configuration of any debounce period.
    /// @param contactor_1_feedback_type Defines the logic behind the feedback (no = normally open, nc = normally close,
    /// none = no feedback).
    /// @param relay_2_name The name of the second relay and its feedback as labeled on hardware.
    /// @param relay_2_actuator_gpio_line_name The name of the GPIO line which switches the second relay on/off.
    /// @param relay_2_feedback_gpio_line_name The name of the GPIO line to which the feedback/sense signal of relay 2
    /// is connected to.
    /// @param relay_2_feedback_gpio_deboucne_us The debounce period which should be configured on the GPIO line in
    /// microseconds. Pass zero to skip the configuration of any debounce period.
    /// @param contactor_2_feedback_type Defines the logic behind the feedback (no = normally open, nc = normally close,
    /// none = no feedback).
    /// @param switch_3ph1ph_enabled True whether relay 2 should be used for phase switching, relay 2 is ignored
    /// otherwise.
    CbTarragonContactorControl(const std::string& relay_1_name, const std::string& relay_1_actuator_gpio_line_name,
                               const std::string& relay_1_feedback_gpio_line_name,
                               const unsigned int relay_1_gpio_debounce_us,
                               const std::string& contactor_1_feedback_type, const std::string& relay_2_name,
                               const std::string& relay_2_actuator_gpio_line_name,
                               const std::string& relay_2_feedback_gpio_line_name,
                               const unsigned int relay_2_gpio_debounce_us,
                               const std::string& contactor_2_feedback_type, bool switch_3ph1ph_enabled);

    /// @brief Get the state of one or both contactors ('actual' or 'target').
    ContactorState get_state(StateType state) const;

    /// @brief Set the target state of one or both contactors.
    /// @param target_state The desired contactor state (OPEN or CLOSED).
    void set_target_state(ContactorState target_state);

    /// @brief Set the actual observed state of the contactor(s).
    /// @param actual_state The observed contactor state (OPEN or CLOSED).
    void set_actual_state(ContactorState actual_state);

    /// @brief Update the number of phases in operation.
    void update_actual_phase_count();

    /// @brief Get the number of phases in operation.
    /// @return number of phases
    ///         (0 = Error: no phases in operation, 1 = 1-phase, 3 = 3-phase).
    int get_actual_phase_count() const;

    /// @brief Get the target number of phases.
    /// @return number of phases
    ///         (1 = 1-phase, 3 = 3-phase).
    int get_target_phase_count() const;

    /// @brief Switch between 3-phase and 1-phase operation mode.
    /// @param use_3phases True if 3 phases shall be used, false otherwise.
    void switch_phase_count(bool use_3phases);

    /// @brief Return the timestamp at which a new state was set to the actuator
    std::chrono::time_point<std::chrono::steady_clock> get_new_target_state_ts() const;

    /// @brief Return the status of the flag marking the start of observation
    ///        of the timestamp at which a new state was set to the actuator.
    bool get_is_new_target_state_set() const;

    /// @brief Reset the flag is_new_target_state_set.
    void reset_is_new_target_state_set();

    /// @brief Return the timestamp at which last actual state change from closed to
    ///        open has occurred.
    std::chrono::time_point<std::chrono::steady_clock> get_last_actual_state_open_ts() const;

    /// @brief Determine if switching on is allowed or not to prevent relay wear.
    /// @Return `true` if switch on allowed, `false` otherwise.
    bool is_switch_on_allowed();

    /// @brief Return the value of the flag delay_contactor_close.
    bool get_delay_contactor_close() const;

    /// @brief Reset the flag delay_contactor_close.
    void reset_delay_contactor_close();

    /// @brief Determine if there exists contactor(s) error (0 or 1).
    bool is_error_state() const;

    /// @brief Wait for new events (GPIO interrupts) occurring on the relays feedback.
    /// @param duration The maximum duration to wait for events to occur.
    /// @return '0' if no events, '1' if events occurred.
    bool wait_for_events(std::chrono::milliseconds duration);

    /// @brief Wait for new events (GPIO interrupts) occurring on the relays feedback.
    /// @return '0' it is a contactor opened event, '1' if it is a contactor closed event.
    bool read_events();

private:
    /// @brief Object representing the first relay on the Tarragon board.
    CbTarragonRelay relay_1;

    /// @brief Object representing the second relay on the Tarragon board.
    ///        This is optional as it might happen that the second relay is
    ///        used for other purposes other than 3-phase operation.
    std::optional<CbTarragonRelay> relay_2;

    /// @brief nc = normally close, no = normally open, none = no feedback.
    std::string contactor_1_feedback_type;

    /// @brief nc = normally close, no = normally open, none = no feedback.
    std::string contactor_2_feedback_type;

    /// @brief Actual state of the contactor(s).
    ContactorState actual_state;

    /// @brief Target state of the contactor(s).
    ContactorState target_state;

    /// @brief Number of phases in operation (1 or 3).
    int actual_phase_count {0};

    /// @brief Desired number of phases (1 or 3).
    int target_phase_count {3};

    /// @brief Timestamp at which a new state was set to the actuator.
    std::chrono::time_point<std::chrono::steady_clock> new_target_state_ts;

    /// @brief Flag to mark the start of observation of the timestamp
    ///        at which a new state was set to the actuator.
    bool is_new_target_state_set;

    /// @brief Timestamp of the last actual state change from closed to open.
    std::chrono::time_point<std::chrono::steady_clock> last_actual_state_open_ts;

    /// @brief Flag to determine if closing the contactor should be delayed.
    bool delay_contactor_close;

    /// @brief Return current state based on evaluation of current GPIOs.
    ContactorState get_current_state();
};

/// @brief Feeds a string representation of the given CbTarragonContactorControl
///        instance into an output stream.
/// @return A reference to the output stream operated on.
std::ostream& operator<<(std::ostream& os, const CbTarragonContactorControl& cc);
