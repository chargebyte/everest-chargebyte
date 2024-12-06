#include <gtest/gtest.h>

/* Files under test */
#include <CPUtils.hpp>

using namespace types::cb_board_support;

CPUtils::cp_state_signal_side cp_state {CPState::A,
                                        CPState::A,
                                        CPState::A,
                                        0};

/* Unit tests for function cp_state_change */
/// @brief Test 1: Stay in CP state A: No detection of a CP state change
TEST(check_for_cp_state_changes, Test_1) {
    cp_state.measured_state = CPState::A;
    EXPECT_FALSE(CPUtils::check_for_cp_state_changes(cp_state));
}

/// @brief Test 2: Change CP state to 'B': First detection shall not lead to a detected CP change
TEST(check_for_cp_state_changes, Test_2) {
    cp_state.previous_state = CPState::A;
    cp_state.current_state = CPState::A;
    cp_state.measured_state = CPState::B;
    EXPECT_FALSE(CPUtils::check_for_cp_state_changes(cp_state));
}

/// @brief Test 3: Stay in 'B': Second detection shall lead to a detected CP change
TEST(check_for_cp_state_changes, Test_3) {
    cp_state.previous_state = CPState::A;
    cp_state.current_state = CPState::B;
    cp_state.measured_state = CPState::B;
    EXPECT_TRUE(CPUtils::check_for_cp_state_changes(cp_state));
    EXPECT_EQ(cp_state.current_state, CPState::B);
}

/// @brief Test 4: Stay in 'B': Third detection shall not lead to a detected CP change
TEST(check_for_cp_state_changes, Test_4) {
    cp_state.previous_state = CPState::B;
    cp_state.current_state = CPState::B;
    cp_state.measured_state = CPState::B;
    EXPECT_FALSE(CPUtils::check_for_cp_state_changes(cp_state));;
}

/// @brief Test 5: Configure single 'A' disturbance: Shall not lead to a detected CP change
TEST(check_for_cp_state_changes, Test_5) {
    cp_state.previous_state = CPState::B;
    cp_state.current_state = CPState::B;
    cp_state.measured_state = CPState::A;

    EXPECT_FALSE(CPUtils::check_for_cp_state_changes(cp_state));
    cp_state.measured_state = CPState::B;
    EXPECT_FALSE(CPUtils::check_for_cp_state_changes(cp_state));
    cp_state.measured_state = CPState::B;
    EXPECT_FALSE(CPUtils::check_for_cp_state_changes(cp_state));
}

/// @brief Test 6: Configure two 'A' disturbances: Shall not lead to a detected CP change
TEST(check_for_cp_state_changes, Test_6) {
    cp_state.previous_state = CPState::B;
    cp_state.current_state = CPState::B;
    cp_state.measured_state = CPState::A;

    EXPECT_FALSE(CPUtils::check_for_cp_state_changes(cp_state));
    cp_state.measured_state = CPState::B;
    EXPECT_FALSE(CPUtils::check_for_cp_state_changes(cp_state));
    cp_state.measured_state = CPState::A;
    EXPECT_FALSE(CPUtils::check_for_cp_state_changes( cp_state));
    cp_state.measured_state = CPState::B;
    EXPECT_FALSE(CPUtils::check_for_cp_state_changes(cp_state));
    cp_state.measured_state = CPState::B;
    EXPECT_FALSE(CPUtils::check_for_cp_state_changes(cp_state));
}

/* Test 7: Configure 'A' after the first 'A' disturbance: 
 * Shall lead to a detected CP change after the disturbance */
/// @brief Test 7: Configure 'A' after the first 'A' disturbance. Shall lead to a detected CP change after the
///                disturbance
TEST(check_for_cp_state_changes, Test_7) {
    cp_state.previous_state = CPState::B;
    cp_state.current_state = CPState::B;
    cp_state.measured_state = CPState::A;

    EXPECT_FALSE(CPUtils::check_for_cp_state_changes(cp_state));
    cp_state.measured_state = CPState::B;
    EXPECT_FALSE(CPUtils::check_for_cp_state_changes(cp_state));
    cp_state.measured_state = CPState::A;
    EXPECT_FALSE(CPUtils::check_for_cp_state_changes(cp_state));
    cp_state.measured_state = CPState::A;
    EXPECT_TRUE(CPUtils::check_for_cp_state_changes (cp_state));
    EXPECT_EQ(cp_state.current_state, CPState::A);
    cp_state.measured_state = CPState::A;
    EXPECT_FALSE(CPUtils::check_for_cp_state_changes(cp_state));
}
