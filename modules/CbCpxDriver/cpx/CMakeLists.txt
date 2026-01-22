set(CPX_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" PARENT_SCOPE)

add_library(cpx STATIC)

target_sources(cpx
    PRIVATE
        "CbCpx.cpp"
        "can_interface/can.c"
)

get_target_property(GENERATED_INCLUDE_DIR generate_cpp_files EVEREST_GENERATED_INCLUDE_DIR)

target_include_directories(cpx
    PRIVATE 
        ${GENERATED_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/../everest-framework/include
        ${CMAKE_SOURCE_DIR}/build/_deps/nlohmann_json_schema_validator-src/src
)

target_include_directories(cpx PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_dependencies(cpx generate_cpp_files)

target_link_libraries(cpx
    PRIVATE
        Pal::Sigslot
        nlohmann_json::nlohmann_json
        everest::log
        everest::sqlite
)
