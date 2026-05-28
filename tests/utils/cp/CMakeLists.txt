
file(GLOB TEST_SOURCES "*.cpp")

add_executable(CP_tests ${TEST_SOURCES})
enable_testing()

find_package(GTest REQUIRED)

target_link_libraries(CP_tests PRIVATE
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
    chargebyte::utils
)

add_test(CP_tests CP_tests)
