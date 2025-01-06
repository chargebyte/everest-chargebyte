set_property(GLOBAL PROPERTY CP_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

add_library(cp STATIC)
add_library(utils::cp ALIAS cp)

target_sources(cp
    PRIVATE
        "CPUtils.cpp"
)

target_include_directories(cp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

get_target_property(GENERATED_INCLUDE_DIR generate_cpp_files EVEREST_GENERATED_INCLUDE_DIR)

add_dependencies(cp generate_cpp_files)

target_include_directories(cp PUBLIC 
    ${GTEST_INCLUDE_DIRS}
    ${GENERATED_INCLUDE_DIR}
)

target_link_libraries(cp
    PUBLIC
        nlohmann_json::nlohmann_json
        everest::framework
)
