
file(GLOB TEST_SOURCES "*.cpp")

add_executable(CP_tests ${TEST_SOURCES})
enable_testing()

set(INCLUDE_DIR 
    "${PROJECT_SOURCE_DIR}/modules/CbTarragonDriver"
    "${PROJECT_SOURCE_DIR}/modules/CbTarragonDriver/tarragon"
    "${PROJECT_SOURCE_DIR}/modules/CbTarragonDriver/evse_board_support"
    "${PROJECT_SOURCE_DIR}/tests/utils/cp")

get_target_property(GENERATED_INCLUDE_DIR generate_cpp_files EVEREST_GENERATED_INCLUDE_DIR)

get_property(CP_INCLUDE_DIR GLOBAL PROPERTY CP_INCLUDE_DIR)

target_include_directories(CP_tests PUBLIC 
    ${GTEST_INCLUDE_DIRS}
    ${INCLUDE_DIR}
    ${GENERATED_INCLUDE_DIR}
    ${CP_INCLUDE_DIR}
)

find_package(GTest REQUIRED)

target_link_libraries(CP_tests PRIVATE
    ${GTEST_LIBRARIES} 
    ${GTEST_MAIN_LIBRARIES} 
    everest::timer
    ${CMAKE_DL_LIBS}
    everest::log
    everest::framework
    pthread
#    nlohmann_json::nlohmann_json
    date::date
    date::date-tz
    utils::cp
)

add_test(CP_tests CP_tests)
