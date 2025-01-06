#include <gtest/gtest.h>

/* Files under test */
#include <CPUtils.hpp>

using namespace CPUtils;

/// @brief Test the voltage_to_state function
/// @details Test the voltage_to_state function with different voltages and previous states
///          to check if the correct state is returned.
TEST(voltage_to_state, VoltageToState) {
    using namespace types::cb_board_support;

    EXPECT_EQ(voltage_to_state(13001 /*mV*/, CPState::A), CPState::PilotFault);
    EXPECT_EQ(voltage_to_state(13000 /*mV*/, CPState::A), CPState::A);
    EXPECT_EQ(voltage_to_state(11001 /*mV*/, CPState::A), CPState::A);
    EXPECT_EQ(voltage_to_state(11000 /*mV*/, CPState::A), CPState::A);
    EXPECT_EQ(voltage_to_state(10001 /*mV*/, CPState::B), CPState::B);
    EXPECT_EQ(voltage_to_state(10000 /*mV*/, CPState::A), CPState::A);
    EXPECT_EQ(voltage_to_state(8000 /*mV*/, CPState::A), CPState::B);
    EXPECT_EQ(voltage_to_state(7999 /*mV*/, CPState::A), CPState::B);
    EXPECT_EQ(voltage_to_state(7000 /*mV*/, CPState::A), CPState::B);
    EXPECT_EQ(voltage_to_state(7000 /*mV*/, CPState::C), CPState::C);
    EXPECT_EQ(voltage_to_state(6999 /*mV*/, CPState::A), CPState::C);
    EXPECT_EQ(voltage_to_state(5000 /*mV*/, CPState::A), CPState::C);
    EXPECT_EQ(voltage_to_state(4999 /*mV*/, CPState::D), CPState::D);
    EXPECT_EQ(voltage_to_state(4000 /*mV*/, CPState::D), CPState::D);
    EXPECT_EQ(voltage_to_state(4000 /*mV*/, CPState::B), CPState::C);
    EXPECT_EQ(voltage_to_state(4000 /*mV*/, CPState::A), CPState::C);
    EXPECT_EQ(voltage_to_state(2000 /*mV*/, CPState::A), CPState::D);
    EXPECT_EQ(voltage_to_state(1999 /*mV*/, CPState::A), CPState::E);
    EXPECT_EQ(voltage_to_state(1999 /*mV*/, CPState::D), CPState::E);
    EXPECT_EQ(voltage_to_state(-1000 /*mV*/, CPState::F), CPState::E);
    EXPECT_EQ(voltage_to_state(-1001 /*mV*/, CPState::F), CPState::PilotFault);
    EXPECT_EQ(voltage_to_state(-2000 /*mV*/, CPState::A), CPState::E);
    EXPECT_EQ(voltage_to_state(-2000 /*mV*/, CPState::D), CPState::E);
    EXPECT_EQ(voltage_to_state(-2000 /*mV*/, CPState::F), CPState::PilotFault);
    EXPECT_EQ(voltage_to_state(-2000 /*mV*/, CPState::PilotFault), CPState::PilotFault);
    EXPECT_EQ(voltage_to_state(-10000 /*mV*/, CPState::F), CPState::PilotFault);
    EXPECT_EQ(voltage_to_state(-10001 /*mV*/, CPState::F), CPState::F);
    EXPECT_EQ(voltage_to_state(-11000 /*mV*/, CPState::F), CPState::F);
    EXPECT_EQ(voltage_to_state(-11000 /*mV*/, CPState::E), CPState::PilotFault);
    EXPECT_EQ(voltage_to_state(-11001 /*mV*/, CPState::E), CPState::F);
    EXPECT_EQ(voltage_to_state(-13000 /*mV*/, CPState::E), CPState::F);
    EXPECT_EQ(voltage_to_state(-13001 /*mV*/, CPState::E), CPState::PilotFault);
}
