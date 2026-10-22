# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 chargebyte GmbH
find_package(GTest REQUIRED)

add_executable(CbGPIOChargePause_tests test_charge_pause.cpp ../CbGPIOChargePause.cpp)
target_include_directories(CbGPIOChargePause_tests PRIVATE
    ..
    $<TARGET_PROPERTY:CbGPIOChargePause,INCLUDE_DIRECTORIES>
)
target_link_libraries(CbGPIOChargePause_tests PRIVATE
    GTest::gtest_main
    $<TARGET_PROPERTY:CbGPIOChargePause,LINK_LIBRARIES>
)
add_dependencies(CbGPIOChargePause_tests generate_cpp_files)
add_test(NAME CbGPIOChargePause COMMAND CbGPIOChargePause_tests)
set_tests_properties(CbGPIOChargePause PROPERTIES TIMEOUT 15)
