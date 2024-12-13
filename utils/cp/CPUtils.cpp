#include <generated/types/cb_board_support.hpp>

#include "CPUtils.hpp"

types::cb_board_support::CPState CPUtils::voltage_to_state(int voltage,
                                                           types::cb_board_support::CPState previous_state) {
    // The following thresholds mostly based on IEC 61851-1
    // Table A.4 System states detected by the charging station.
    if (voltage > 13000 /* mV */) /* > 13 V */
        return types::cb_board_support::CPState::PilotFault;

    if (voltage >= 11000 /* mV */) /* 11 V <= x <= 13 V */
        return types::cb_board_support::CPState::A;

    if (voltage >= 10000 /* mV */) { /* 10 V <= x < 11 V */
        switch (previous_state) {
        case types::cb_board_support::CPState::A:
            return types::cb_board_support::CPState::A;
        default:
            return types::cb_board_support::CPState::B;
        }
    }

    if (voltage >= 8000 /* mV */) /* 8 V <= x < 10 V */
        return types::cb_board_support::CPState::B;

    if (voltage >= 7000 /* mV */) { /* 7 V <= x < 8 V */
        switch (previous_state) {
        case types::cb_board_support::CPState::A:
        case types::cb_board_support::CPState::B:
            return types::cb_board_support::CPState::B;
        default:
            return types::cb_board_support::CPState::C;
        }
    }

    if (voltage >= 5000 /* mV */) /* 5 V <= x < 7 V */
        return types::cb_board_support::CPState::C;

    if (voltage >= 4000 /* mV */) { /* 4 V <= x < 5 V */
        switch (previous_state) {
        case types::cb_board_support::CPState::A:
        case types::cb_board_support::CPState::B:
        case types::cb_board_support::CPState::C:
            return types::cb_board_support::CPState::C;
        default:
            return types::cb_board_support::CPState::D;
        }
    }

    if (voltage >= CP_STATE_D_LOWER_THRESHOLD) /* 2 V <= x < 4 V */
        return types::cb_board_support::CPState::D;

    if (voltage >= -1000 /* mV */) /* -1 V <= x < 2 V */
        return types::cb_board_support::CPState::E;

    if (voltage >= -2000 /* mV */) { /* -2 V <= x < -1 V (not standard) */
        switch (previous_state) {
        case types::cb_board_support::CPState::A:
        case types::cb_board_support::CPState::B:
        case types::cb_board_support::CPState::C:
        case types::cb_board_support::CPState::D:
        case types::cb_board_support::CPState::E:
            return types::cb_board_support::CPState::E;
        default:
            return types::cb_board_support::CPState::PilotFault;
        }
    }

    if (voltage >= -10000 /* mV */) /* -10 V <= x < -2 V */
        return types::cb_board_support::CPState::PilotFault;

    if (voltage >= -11000 /* mV */) { /* -11 V <= x < -10 V (not standard) */
        return previous_state == types::cb_board_support::CPState::F ? types::cb_board_support::CPState::F
                                                                     : types::cb_board_support::CPState::PilotFault;
    }

    if (voltage >= -13000 /* mV */) /* -13 V <= x < -11 V */
        return types::cb_board_support::CPState::F;

    return types::cb_board_support::CPState::PilotFault; /* < -13 V */
}

bool CPUtils::is_cp_short_fault(const double& duty_cycle, const int& voltage_neg_side, const int& voltage_pos_side) {
    // Check for CP short circuit
    // If 100 % or 0% duty cycle: If a state transitition to E is detected
    if ((duty_cycle == 100.0 || duty_cycle == 0.0) &&
        (voltage_pos_side < CP_STATE_D_LOWER_THRESHOLD) &&
        (voltage_neg_side > -10000 /* mV */)) {
        return true;
    }
    // If nominal duty cycle: If positive side below 2V and the difference between the absolute values of the voltages
    //                        from negative and positive is smaller or equal then 1,2V
    if (is_nominal_duty_cycle(duty_cycle) &&
        (voltage_pos_side < CP_STATE_D_LOWER_THRESHOLD) &&
        (abs(voltage_pos_side + voltage_neg_side) <= CP_ERROR_VOLTAGE_MAX_DIFF)) {
        return true;
    }

    return false;
}

bool CPUtils::is_ventilation_fault(const types::cb_board_support::CPState& current_cp_state, const int& voltage_neg_side) {
    // Check for ventilation fault
    // CP state D is not supported
    return (current_cp_state == types::cb_board_support::CPState::D &&
            voltage_neg_side <= -10000);
}

bool CPUtils::is_diode_fault(const double& duty_cycle, const int& voltage_neg_side, const int& voltage_pos_side) {
    // Check if a diode fault has occurred
    // nominal duty cycle: If positive side above 2V and the difference between the absolute values of the voltages
    //                     from negative and positive is smaller or equal then 1,2V 0% & 100% duty cycle: not
    //                     possible to detect a diode fault
    return ((is_nominal_duty_cycle(duty_cycle) &&
            (voltage_pos_side > CP_STATE_D_LOWER_THRESHOLD) && 
            (abs(voltage_pos_side + voltage_neg_side) <= CP_ERROR_VOLTAGE_MAX_DIFF)));
}

bool CPUtils::is_pilot_fault(const types::cb_board_support::CPState& current_cp_state) {
    // Check for pilot fault
    // If the CP state is out of range, a pilot fault is detected.
    return (current_cp_state == types::cb_board_support::CPState::PilotFault);
}

bool CPUtils::check_for_cp_errors(cp_state_errors& cp_errors,
                                  types::cb_board_support::CPState& current_cp_state,
                                  const double& duty_cycle,
                                  const int& voltage_neg_side,
                                  const int& voltage_pos_side) {
    bool is_error {false};
    // Check for CP short fault
    if (is_cp_short_fault(duty_cycle, voltage_neg_side, voltage_pos_side)) {
        cp_errors.cp_short_fault.is_active = true;
        is_error = true;
        current_cp_state = types::cb_board_support::CPState::E;
    }
    else {
        cp_errors.cp_short_fault.is_active = false;
    }

    // Check for ventilation fault
    if (is_ventilation_fault(current_cp_state, voltage_neg_side)) {
        cp_errors.ventilation_fault.is_active = true;
        is_error = true;
    }
    else {
        cp_errors.ventilation_fault.is_active = false;
    }

    // Check for diode fault
    // Check if the state is disconnected. This is necessary to clear the diode fault
    bool is_disconnected = (current_cp_state == types::cb_board_support::CPState::A);

    // Clear the diode fault error only if the CP is disconnected (11V < U_CP+ < 13V), otherwise a swing between diode
    // fault and non diode fault is possible if the duty cycle changes between 100% and nominal duty cycle.
    if (cp_errors.diode_fault.is_active && is_disconnected) {
        cp_errors.diode_fault.is_active = false;
    }
    else if ((cp_errors.diode_fault.is_active == false) &&
               is_diode_fault(duty_cycle, voltage_neg_side, voltage_pos_side)) {
        cp_errors.diode_fault.is_active = true;
        is_error = true;
    }
    // Set error if diode fault is already active
    else if (cp_errors.diode_fault.is_active) {
        is_error = true;
    }
    // Check for pilot fault
    // The pilot fault is cleared if the CP state is changes back to a valid state.
    // Only notify a pilot fault if no diode fault is detected.
    else if (is_pilot_fault(current_cp_state)) {
        cp_errors.pilot_fault.is_active = true;
        is_error = true;
    }
    else {
        cp_errors.pilot_fault.is_active = false;
    }

    return is_error;
}

bool CPUtils::check_for_cp_state_changes(struct cp_state_signal_side& signal_side,
                                         const types::cb_board_support::CPState& measured_cp_state) {
    bool rv {false};
    // CP state change is only detected if measured state is equal to the previous measured state and
    // the actually detected state is not equal to the measured state
    if ((signal_side.measured_state_t1 == measured_cp_state) &&
        (signal_side.detected_state != measured_cp_state)) {
            signal_side.detected_state = measured_cp_state;
            rv = true;
    // Otherwise, try to filter single disturbances by checking if the second measured state is surrounded
    // by two equal states (e.g. A -> B -> A)
    } else if ((measured_cp_state == signal_side.measured_state_t0) &&
               (measured_cp_state != signal_side.measured_state_t1)) {
        EVLOG_warning << "CP state change from " << signal_side.measured_state_t0 << " to " << signal_side.measured_state_t1
                      << " suppressed";
    }
    // Update measured states
    signal_side.measured_state_t0 = signal_side.measured_state_t1;
    signal_side.measured_state_t1 = measured_cp_state;

    return rv;
}
