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

    if (voltage >= 2000 /* mV */) /* 2 V <= x < 4 V */
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

bool CPUtils::check_for_cp_errors(cp_state_errors& cp_errors,
                                  types::cb_board_support::CPState& current_cp_state,
                                  const double& duty_cycle,
                                  const cp_state_signal_side& negative_side,
                                  const cp_state_signal_side& positive_side) {
    bool is_error {false};
    // Check for CP errors
    // Check if a diode fault has occurred
    // nominal duty cycle: If positive side above 2V and the difference between the absolute values of the voltages
    //                     from negative and positive is smaller or equal then 1,2V 0% & 100% duty cycle: not
    //                     possible to detect a diode fault
    if (is_nominal_duty_cycle(duty_cycle) &&
        (positive_side.voltage > 2000 /* mV */) &&
        (abs(positive_side.voltage + negative_side.voltage) <= 1200 /* mV */)) {
        if (cp_errors.diode_fault.is_active == false) {
            cp_errors.diode_fault.is_active = true;
            is_error = true;
        }
        else {
            cp_errors.diode_fault.is_active = false;
        }
    }
    // If no diode fault is detected and the PilotFault is active, we assume the CP state out of range
    else if (negative_side.current_state == types::cb_board_support::CPState::PilotFault ||
             positive_side.current_state == types::cb_board_support::CPState::PilotFault) {
        if (cp_errors.pilot_fault.is_active == false) {
            cp_errors.pilot_fault.is_active = true;
            is_error = true;
            current_cp_state = types::cb_board_support::CPState::PilotFault;
        }
        else {
            cp_errors.pilot_fault.is_active = false;
        }
    }

    // Check for CP short circuit
    // If 100 % duty cycle:  If a state transitition to E is detected
    // If nominal duty cycle: If positive side below 2V and the difference between the absolute values of the voltages
    //                        from negative and positive is smaller or equal then 1,2V
    // If 0 % duty cycle: If negative side above -10V
    if (((duty_cycle == 100.0) && (positive_side.voltage < 2000 /* mV */)) ||
        ((is_nominal_duty_cycle(duty_cycle)) && (positive_side.voltage < 2000 /* mV */) &&
        (abs(positive_side.voltage + negative_side.voltage) <= 1200 /* mV */) ) ||
        ((duty_cycle == 0.0) && (negative_side.voltage > -10000 /* mV */))) {
        if (cp_errors.cp_short_fault.is_active == false) {
            cp_errors.cp_short_fault.is_active = true;
            is_error = true;
            current_cp_state = types::cb_board_support::CPState::E;
        }
        /* Only clear CP short error in case of CPState::A (Unplugged) */
        else if (positive_side.voltage >= 11000 /* mV */) {
            cp_errors.cp_short_fault.is_active = false;
        }
    }

    // Check for ventilation fault
    // CP state D is not supported
    if (positive_side.current_state == types::cb_board_support::CPState::D) {
        if (cp_errors.ventilation_fault.is_active == false) {
            cp_errors.ventilation_fault.is_active = true;
            is_error = true;
        }
        else {
            cp_errors.ventilation_fault.is_active = false;
        }
    }

    return is_error;
}

bool CPUtils::check_for_cp_state_changes(struct cp_state_signal_side& signal_side) {
    bool rv {false};

    // CP state is only detected if the new state is different from the previous one (first condition).
    // Additionally, to filter simple disturbances, a new state must be detected twice before notifying it (second
    // condition). For that, we need at least two CP state measurements (third condition)

    if (signal_side.previous_state != signal_side.measured_state &&
        signal_side.current_state == signal_side.measured_state) {

        // update the previous state
        signal_side.previous_state = signal_side.current_state;
        rv = true;

    } else if (signal_side.measured_state == signal_side.previous_state &&
               signal_side.measured_state != signal_side.current_state) {
        EVLOG_warning << "CP state change from " << signal_side.previous_state << " to " << signal_side.current_state
                      << " suppressed";
    }

    signal_side.current_state = signal_side.measured_state;

    return rv;
}