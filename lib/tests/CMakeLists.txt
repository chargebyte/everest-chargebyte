set(TEST_TARGET_NAME ${PROJECT_NAME}_cb-utils_tests)

add_executable(${TEST_TARGET_NAME}
    cb-utils_test.cpp
)

target_link_libraries(${TEST_TARGET_NAME}
    PRIVATE
        GTest::gmock_main
        GTest::gtest_main
        chargebyte::utils
)

include(GoogleTest)
gtest_discover_tests(${TEST_TARGET_NAME})

ev_register_test_target(${TEST_TARGET_NAME})
