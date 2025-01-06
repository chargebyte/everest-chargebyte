#include <gtest/gtest.h>

/* Files under test */
#include "tarragon/CbTarragonCPADC.hpp"

/// @brief Test the calc_voltage function
/// @details Test the calc_voltage function with different ADC values
///          to check if the correct voltage is returned.
TEST(cp, calc_voltage)
{
    /*
     * negative detector:
     * CP         ; V 3p3    ; ADC
     * -11,954516 ; 0,423645 ; 527
     * -8,7518900 ; 0,757881 ; 940
     *
     * positive detector
     * 11,830832 ; 2,9059572 ; 3606
     * 2,6012671 ; 1,9427314 ; 2411
     */
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(-1000), -23726);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(0), -16013);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(1), -16013);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(2), -16004);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(3), -15994);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(4), -15985);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(5), -15975);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(6), -15975);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(7), -15966);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(8), -15956);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(9), -15946);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(10), -15937);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(527), -11951);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(940), -8760);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(1024), -8108);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(2048), -203);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(2074), -2);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(2075), 7);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(2411), 2595);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(3072), 7702);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(3606), 11822);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(4090), 15559);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(4091), 15569);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(4092), 15578);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(4093), 15588);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(4094), 15597);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(4095), 15607);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(4096), 15607);
    EXPECT_EQ(CbTarragonCPADC::calc_voltage(4097), 15616);
}
