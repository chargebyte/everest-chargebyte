set(CHARGESOM_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" PARENT_SCOPE)

add_library(chargesom STATIC)

target_sources(chargesom
    PRIVATE
        "CbChargeSOM.cpp"
)

get_target_property(GENERATED_INCLUDE_DIR generate_cpp_files EVEREST_GENERATED_INCLUDE_DIR)

get_property(LINUX_HW_INCLUDE_DIR GLOBAL PROPERTY LINUX_HW_INCLUDE_DIR)

target_include_directories(chargesom
    PRIVATE
        ${GENERATED_INCLUDE_DIR}
        ${LINUX_HW_INCLUDE_DIR}
        ${LIBGPIOD_INCLUDE_DIR}
)

add_dependencies(chargesom generate_cpp_files)

target_link_libraries(chargesom
    PRIVATE
        Pal::Sigslot
        nlohmann_json::nlohmann_json
        everest::log
        utils::linux_hw
        ra-utils
        ${LIBGPIOD_LIBRARIES}
)
