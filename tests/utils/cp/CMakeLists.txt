
file(GLOB TEST_SOURCES "*.cpp")

add_executable(CP_tests ${TEST_SOURCES})
enable_testing()

get_property(CP_INCLUDE_DIR GLOBAL PROPERTY CP_INCLUDE_DIR)

target_include_directories(CP_tests PUBLIC 
    ${CP_INCLUDE_DIR}
)

find_package(GTest REQUIRED)

target_link_libraries(CP_tests PRIVATE
    ${GTEST_LIBRARIES} 
    ${GTEST_MAIN_LIBRARIES} 
    utils::cp
)

add_test(CP_tests CP_tests)
