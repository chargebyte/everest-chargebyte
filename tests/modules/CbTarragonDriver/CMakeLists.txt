
file(GLOB TEST_SOURCES "*.cpp")

add_executable(CbTarragonDriver_tests ${TEST_SOURCES})
enable_testing()

set(INCLUDE_DIR 
    "${PROJECT_SOURCE_DIR}/modules/CbTarragonDriver")

get_target_property(GENERATED_INCLUDE_DIR generate_cpp_files EVEREST_GENERATED_INCLUDE_DIR)

get_property(LINUX_HW_INCLUDE_DIR GLOBAL PROPERTY LINUX_HW_INCLUDE_DIR)

target_include_directories(CbTarragonDriver_tests PUBLIC 
    ${INCLUDE_DIR}
    ${LINUX_HW_INCLUDE_DIR}
)

find_package(GTest REQUIRED)

target_link_libraries(CbTarragonDriver_tests PRIVATE
    ${GTEST_LIBRARIES} 
    ${GTEST_MAIN_LIBRARIES} 
    tarragon
)

add_test(CbTarragonDriver_tests CbTarragonDriver_tests)
