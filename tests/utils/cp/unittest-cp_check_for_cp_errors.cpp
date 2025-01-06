#include <gtest/gtest.h>

/* Files under test */
#include <CPUtils.hpp>

using namespace types::cb_board_support;
using namespace CPUtils;

/// @brief check_for_cp_state_changes - good test cases 
/// @details Test the check_for_cp_state_changes function with different previous, current and measured states
///          to check if the correct state change is detected.
TEST(CPUtilsTest, check_for_cp_errors_good_cases) {
    // Fist all good cases
    cp_state_errors cp_errors {};
    // Good case: EV disconnected
    CPState current_cp_state = CPState::A;
    EXPECT_FALSE(check_for_cp_errors(cp_errors, current_cp_state, 100.0, 0 /*mV*/, 12999 /*mV*/));
    EXPECT_EQ(current_cp_state, CPState::A);
    EXPECT_FALSE(check_for_cp_errors(cp_errors, current_cp_state, 100.0, 0 /*mV*/, 10001 /*mV*/));
    EXPECT_EQ(current_cp_state, CPState::A);

    // Good case: EV connected, 100% duty cycle
    current_cp_state = CPState::B;
    EXPECT_FALSE(check_for_cp_errors(cp_errors, current_cp_state, 100.0, 0 /*mV*/, 10000 /*mV*/));
    EXPECT_EQ(current_cp_state, CPState::B);
    EXPECT_FALSE(check_for_cp_errors(cp_errors, current_cp_state, 100.0, 0 /*mV*/, 7000 /*mV*/));
    EXPECT_EQ(current_cp_state, CPState::B);

    // Good case: EV connected, nominal duty cycle
    current_cp_state = CPState::B;
    EXPECT_FALSE(check_for_cp_errors(cp_errors, current_cp_state, 50.0, -12000 /*mV*/, 10000 /*mV*/));
    EXPECT_EQ(current_cp_state, CPState::B);
    EXPECT_FALSE(check_for_cp_errors(cp_errors, current_cp_state, 100.0, -12000 /*mV*/, 7000 /*mV*/));
    EXPECT_EQ(current_cp_state, CPState::B);

    // Good case: EV connected, nominal duty cycle CP state C
    current_cp_state = CPState::C;
    EXPECT_FALSE(check_for_cp_errors(cp_errors, current_cp_state, 0.0, -12000 /*mV*/, 7000 /*mV*/));
    EXPECT_EQ(current_cp_state, CPState::C);
    EXPECT_FALSE(check_for_cp_errors(cp_errors, current_cp_state, 20.0, -12000 /*mV*/, 4000 /*mV*/));
    EXPECT_EQ(current_cp_state, CPState::C);

    // Good case: EV connected, 100% duty cycle CP state C
    EXPECT_FALSE(check_for_cp_errors(cp_errors, current_cp_state, 100.0, -12000 /*mV*/, 5000 /*mV*/));
    EXPECT_EQ(current_cp_state, CPState::C);

    // Good case: EV connected, 0% duty cycle CP state F
    current_cp_state = CPState::F;
    EXPECT_FALSE(check_for_cp_errors(cp_errors, current_cp_state, 0.0, -11000 /*mV*/, 0 /*mV*/));
    EXPECT_EQ(current_cp_state, CPState::F);
    EXPECT_FALSE(check_for_cp_errors(cp_errors, current_cp_state, 100.0, -13000 /*mV*/, 0 /*mV*/));
    EXPECT_EQ(current_cp_state, CPState::F);
}

/// @brief  CPUtilsTest: check_for_cp_errors - Diode fault test cases
/// @details Conditions: nominal duty cycle: If positive side above 2V and the difference between the
///          absolute values of the voltages from negative and positive is smaller or equal then 1,2V
///          0% & 100% duty cycle: not possible to detect a diode fault
TEST(CPUtilsTest, check_for_cp_errors_diode_fault) {
    // Trigger diode fault. Check upper boundary
    cp_state_errors cp_errors {};
    CPState current_cp_state = CPState::E;
    bool is_error = CPUtils::check_for_cp_errors(cp_errors, current_cp_state, 50, -3201 /*mV*/, 2001 /*mV*/);

    EXPECT_TRUE(is_error);
    EXPECT_TRUE(cp_errors.diode_fault.is_active);
    EXPECT_FALSE(cp_errors.pilot_fault.is_active);
    EXPECT_FALSE(cp_errors.ventilation_fault.is_active);
    EXPECT_FALSE(cp_errors.cp_short_fault.is_active);

    // Clear diode fault down boundary
    // Conditions: Should only be cleared if EV is disconnected (11V < U_CP+ < 13V)
    current_cp_state = CPState::A;
    is_error = CPUtils::check_for_cp_errors(cp_errors, current_cp_state, 100, 0 /*mV*/, 11000 /*mV*/);
    EXPECT_FALSE(is_error);
    EXPECT_FALSE(cp_errors.diode_fault.is_active);

    // Trigger diode fault. Check down boundary
    current_cp_state = CPState::B;
    is_error = CPUtils::check_for_cp_errors(cp_errors, current_cp_state, 50, -9799 /*mV*/, 10999 /*mV*/);
    EXPECT_TRUE(is_error);
    EXPECT_TRUE(cp_errors.diode_fault.is_active);
    EXPECT_FALSE(cp_errors.pilot_fault.is_active);
    EXPECT_FALSE(cp_errors.ventilation_fault.is_active);
    EXPECT_FALSE(cp_errors.cp_short_fault.is_active);

    // Clear diode fault upper boundary
    // Conditions: Should only be cleared if EV is disconnected (11V < U_CP+ < 13V)
    // Now try to clear diode fault with valid CP state B
    is_error = CPUtils::check_for_cp_errors(cp_errors, current_cp_state, 50, -12000 /*mV*/, 9000 /*mV*/);
    EXPECT_TRUE(is_error);
    EXPECT_TRUE(cp_errors.diode_fault.is_active);
    EXPECT_FALSE(cp_errors.pilot_fault.is_active);
    EXPECT_FALSE(cp_errors.ventilation_fault.is_active);
    EXPECT_FALSE(cp_errors.cp_short_fault.is_active);    

    // Clear diode fault with CP state A
    current_cp_state = CPState::A;
    is_error = CPUtils::check_for_cp_errors(cp_errors, current_cp_state, 100, 0 /*mV*/, 13000 /*mV*/);
    EXPECT_FALSE(is_error);
    EXPECT_FALSE(cp_errors.diode_fault.is_active);

    // Try to trigger diode fault in CP state A. This should not lead to an error, 
    // because in A we assume the EV is disconnected
    current_cp_state = CPState::A;
    is_error = CPUtils::check_for_cp_errors(cp_errors, current_cp_state, 50, -12000 /*mV*/, 13000 /*mV*/);
    EXPECT_FALSE(is_error);
    EXPECT_FALSE(cp_errors.diode_fault.is_active);
}

/// @brief  CPUtilsTest: check_for_cp_errors - Pilot fault test cases
/// @details Conditions: If the CP state is out of range, a pilot fault is detected.
///          The pilot fault is cleared if the CP state is changes back to a valid state.
///          Only notify a pilot fault if no diode fault is detected.
TEST(CPUtilsTest, check_for_cp_errors_pilot_fault) {
    // Trigger diode fault. Pilot fault should not be detected
    cp_state_errors cp_errors {};
    CPState current_cp_state = CPState::PilotFault;
    bool is_error = CPUtils::check_for_cp_errors(cp_errors, current_cp_state, 50, -3201 /*mV*/, 2001 /*mV*/);
    EXPECT_TRUE(is_error);
    EXPECT_TRUE(cp_errors.diode_fault.is_active);
    EXPECT_FALSE(cp_errors.pilot_fault.is_active);
    // Clear diode fault
    current_cp_state = CPState::A;
    is_error = CPUtils::check_for_cp_errors(cp_errors, current_cp_state, 100, 0 /*mV*/, 13000 /*mV*/);
    EXPECT_FALSE(is_error);
    // Trigger pilot fault
    current_cp_state = CPState::PilotFault;
    is_error = CPUtils::check_for_cp_errors(cp_errors, current_cp_state, 50, 0 /*mV*/, 13001 /*mV*/);
    EXPECT_TRUE(is_error);
    EXPECT_FALSE(cp_errors.diode_fault.is_active);
    EXPECT_TRUE(cp_errors.pilot_fault.is_active);
    // Clear pilot fault by changing to a valid state
    current_cp_state = CPState::B;
    is_error = CPUtils::check_for_cp_errors(cp_errors, current_cp_state, 50, 0 /*mV*/, 9000 /*mV*/);
    EXPECT_FALSE(is_error);
}

/// @brief CPUtilsTest: check_for_cp_errors - CP short fault test cases
/// @details Conditions: 100 % or 0% duty cycle:  If a state transition to E is detected
///          Nominal duty cycle: If positive side below 2V and the difference between the absolute values
///          of the voltages from negative and positive is smaller or equal then 1,2V 
TEST(CPUtilsTest, check_for_cp_errors_CP_short_fault) {
    // Trigger CP short fault. Check upper boundary
    cp_state_errors cp_errors {};
    CPState current_cp_state = CPState::E;
    bool is_error = CPUtils::check_for_cp_errors(cp_errors, current_cp_state, 100, -9999 /*mV*/, 1999 /*mV*/);
    EXPECT_TRUE(is_error);
    EXPECT_FALSE(cp_errors.diode_fault.is_active);
    EXPECT_FALSE(cp_errors.pilot_fault.is_active);
    EXPECT_FALSE(cp_errors.ventilation_fault.is_active);
    EXPECT_TRUE(cp_errors.cp_short_fault.is_active);
    EXPECT_EQ(current_cp_state, CPState::E);

    // Clear CP short with CP state A
    current_cp_state = CPState::A;
    is_error = CPUtils::check_for_cp_errors(cp_errors, current_cp_state, 100, 0 /*mV*/, 12000 /*mV*/);
    EXPECT_FALSE(is_error);

    // Trigger CP short fault. Nominal duty cycle. Upper boundary
    current_cp_state = CPState::E;
    is_error = CPUtils::check_for_cp_errors(cp_errors, current_cp_state, 50, -1200 /*mV*/, 0 /*mV*/);
    EXPECT_TRUE(is_error);
    EXPECT_FALSE(cp_errors.diode_fault.is_active);
    EXPECT_FALSE(cp_errors.pilot_fault.is_active);
    EXPECT_FALSE(cp_errors.ventilation_fault.is_active);
    EXPECT_TRUE(cp_errors.cp_short_fault.is_active);
    EXPECT_EQ(current_cp_state, CPState::E);

    // Clear CP short, now with CP state B
    current_cp_state = CPState::B;
    is_error = CPUtils::check_for_cp_errors(cp_errors, current_cp_state, 50, -12000 /*mV*/, 8000 /*mV*/);
    EXPECT_FALSE(is_error);

    // Trigger CP short fault. Check down boundary
    current_cp_state = CPState::E;
    is_error = CPUtils::check_for_cp_errors(cp_errors, current_cp_state, 50, -799 /*mV*/, 1999 /*mV*/);
    EXPECT_TRUE(is_error);
    EXPECT_FALSE(cp_errors.diode_fault.is_active);
    EXPECT_FALSE(cp_errors.pilot_fault.is_active);
    EXPECT_FALSE(cp_errors.ventilation_fault.is_active);
    EXPECT_TRUE(cp_errors.cp_short_fault.is_active);

    // Clear CP short fault upper boundary
    current_cp_state = CPState::C;
    is_error = CPUtils::check_for_cp_errors(cp_errors, current_cp_state, 50, 0 /*mV*/, 7000 /*mV*/);
    EXPECT_FALSE(is_error);
    EXPECT_FALSE(cp_errors.cp_short_fault.is_active);

    // Trigger CP short fault. Both sides configured to 0V
    current_cp_state = CPState::E;
    is_error = CPUtils::check_for_cp_errors(cp_errors, current_cp_state, 50, 0 /*mV*/, 0 /*mV*/);
    EXPECT_TRUE(is_error);
    EXPECT_FALSE(cp_errors.diode_fault.is_active);
    EXPECT_FALSE(cp_errors.pilot_fault.is_active);
    EXPECT_FALSE(cp_errors.ventilation_fault.is_active);
    EXPECT_TRUE(cp_errors.cp_short_fault.is_active);

    // Clear CP short in CP state B
    current_cp_state = CPState::B;
    is_error = CPUtils::check_for_cp_errors(cp_errors, current_cp_state, 50, 0 /*mV*/, 9000 /*mV*/);
    EXPECT_FALSE(is_error);
    EXPECT_FALSE(cp_errors.cp_short_fault.is_active);
}

/// @brief CPUtilsTest: check_for_cp_errors - Ventilation fault
/// @details Conditions: Check if CP state D is detected as a ventilation fault
TEST(CPUtilsTest, check_for_cp_errors_ventilation_fault) {
    // Trigger ventilation fault
    cp_state_errors cp_errors {};
    CPState current_cp_state = CPState::D;
    bool is_error = CPUtils::check_for_cp_errors(cp_errors, current_cp_state, 50, -12000 /*mV*/, 3999 /*mV*/);
    EXPECT_TRUE(is_error);
    EXPECT_FALSE(cp_errors.diode_fault.is_active);
    EXPECT_FALSE(cp_errors.pilot_fault.is_active);
    EXPECT_TRUE(cp_errors.ventilation_fault.is_active);
    EXPECT_FALSE(cp_errors.cp_short_fault.is_active);

    // Clear ventilation fault
    current_cp_state = CPState::C;
    is_error = CPUtils::check_for_cp_errors(cp_errors, current_cp_state, 50, -12000 /*mV*/, 7000 /*mV*/);
    EXPECT_FALSE(is_error);
    EXPECT_FALSE(cp_errors.ventilation_fault.is_active);
}
