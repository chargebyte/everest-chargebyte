#ifndef CP_UTILS_HPP
#define CP_UTILS_HPP

#include <generated/types/cb_board_support.hpp>
#include <generated/interfaces/evse_board_support/Implementation.hpp>

namespace CPUtils {
    /// @brief Maximum voltage difference between CP+ and CP- for a CP short/diode fault
    static const uint16_t CP_ERROR_VOLTAGE_MAX_DIFF {1200 /*mV*/};
    /// @brief Lower threshold for CP state D
    static const uint16_t CP_STATE_D_LOWER_THRESHOLD {2000 /*mV*/};

    /// @brief Struct to hold the state of the CP signal
    struct cp_state_signal_side {
        /// @brief previous state is what we measured before the last round
        types::cb_board_support::CPState previous_state;

        /// @brief current state is what we measured in the last round
        types::cb_board_support::CPState current_state;

        /// @brief measured state is what we just measured in this round
        types::cb_board_support::CPState measured_state;

        /// @brief the voltage of the just completed measurement
        int voltage;
    };

    /// @brief Struct to hold the state of the CP errors
    struct everest_error {
        std::string type; // The error type as defined in the Everest error model
        std::string sub_type; // The error type as defined in the Everest error model
        std::string message; // The error type as defined in the Everest error model
        Everest::error::Severity severity; // The error severity as defined in the Everest error model
        bool is_active; // 
        bool is_reported;
    };

    /// @brief Struct to hold the different CP errors
    struct cp_state_errors {
        // Note: New error must be added to the errors array below. Please ensure if the error type is already defined,
        //       a unique sub type must be provided.
        everest_error diode_fault {"evse_board_support/DiodeFault", "",
                                   "Diode fault detected.", Everest::error::Severity::High, false, false};
        everest_error pilot_fault {"evse_board_support/MREC14PilotFault", "CPOutOfRange",
                                   "CP voltage is out of range.", Everest::error::Severity::High, false, false};
        everest_error cp_short_fault {"evse_board_support/MREC14PilotFault", "CPShortFault",
                                      "CP short fault detected.", Everest::error::Severity::High, false, false};
        everest_error ventilation_fault {"evse_board_support/VentilationNotAvailable", "",
                                         "Ventilation fault detected.", Everest::error::Severity::High, false, false};

        std::array<std::reference_wrapper<everest_error>, 4> errors = {diode_fault, pilot_fault, cp_short_fault, ventilation_fault};

        auto begin() { return errors.begin(); }
        auto end() { return errors.end(); }
    };

    // Helper methods for CP handling
    /// @brief Helper to map a measured voltage to a CP state (takes hysteresis into account)
    types::cb_board_support::CPState voltage_to_state(int voltage, types::cb_board_support::CPState previous_state);

    /// @brief Check whether the current duty cycle is nominal.
    //  @param duty_cycle The duty cycle to check.
    /// @return True when configured duty cycle is >0 and <100% (nominal duty cycle),
    inline bool is_nominal_duty_cycle(const double& duty_cycle) {
        return 0.0 < duty_cycle && duty_cycle < 100.0;
    }

    /// @brief Helper to check for CP errors
    bool check_for_cp_errors(cp_state_errors& cp_errors,
                             types::cb_board_support::CPState& current_cp_state,
                             const double& duty_cycle,
                             const int& voltage_neg_side,
                             const int& voltage_pos_side);

    /// @brief Helper to determine whether one side of the CP signal caused a CP signal change
    bool check_for_cp_state_changes(struct cp_state_signal_side& signal_side);

    /// @brief Helper to raise errors
    template <typename T, std::size_t N>
    void process_everest_errors(T& obj, const std::array<std::reference_wrapper<everest_error>, N>& errors) {
        // Interate over all errors and raise them if they are active, otherwise clear them if they are reported
        for (const auto& error : errors) {
            auto& error_ref = error.get();
            if (error_ref.is_active && error_ref.is_reported == false) {
                Everest::error::Error error_object = obj.error_factory->create_error(
                    error_ref.type, error_ref.sub_type, error_ref.message, error_ref.severity);
                obj.raise_error(error_object);
                error_ref.is_reported = true;
            }
            else if (error_ref.is_active == false && error_ref.is_reported) {
                obj.clear_error(error_ref.type);
                error_ref.is_reported = false;
            }
        }
    }
}

#endif // CP_UTILS_HPP