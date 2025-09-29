set(CPX_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" PARENT_SCOPE)

# Boost Log + weitere Komponenten finden
find_package(Boost 1.71 REQUIRED COMPONENTS
    log
    log_setup
    system
    thread
    filesystem
    date_time
    regex
)

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
        ${CMAKE_SOURCE_DIR}/../liblog/include
        ${CMAKE_SOURCE_DIR}/build/_deps/nlohmann_json_schema_validator-src/src
        ${CMAKE_SOURCE_DIR}/build/_deps/date-src/include
        ${CMAKE_SOURCE_DIR}/build/_deps/libfmt-src/include
        ${CMAKE_BINARY_DIR}/generated/modules/CbCpxDriver
)

target_include_directories(cpx PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_dependencies(cpx generate_cpp_files)

target_link_libraries(cpx
    PRIVATE
        Pal::Sigslot
        nlohmann_json::nlohmann_json
        Boost::log
        Boost::log_setup
        Boost::system
        Boost::thread
        Boost::filesystem
        Boost::date_time
        Boost::regex
)
