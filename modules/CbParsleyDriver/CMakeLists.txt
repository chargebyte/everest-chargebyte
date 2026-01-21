#
# AUTO GENERATED - MARKED REGIONS WILL BE KEPT
# template version 3
#

# module setup:
#   - ${MODULE_NAME}: module name
ev_setup_cpp_module()

# ev@bcc62523-e22b-41d7-ba2f-825b493a3c97:v1
# insert your custom targets and additional config variables here

find_package(PalSigslot REQUIRED)
find_package(ra-utils REQUIRED)

add_subdirectory("parsley")

set(MODULE_DESCRIPTION "chargebyte's Charge Control Y BSP module")
set(MODULE_VERSION ${PROJECT_VERSION})

# make CMake project variables usable in source code
configure_file(configuration.h.in configuration.h @ONLY)

# create a VERSION file
configure_file(VERSION.in VERSION @ONLY)

target_link_libraries(${MODULE_NAME}
    PRIVATE
        utils::cp
        parsley
        Pal::Sigslot
        nlohmann_json::nlohmann_json
        fmt::fmt
        ra-utils
)

get_property(CP_INCLUDE_DIR GLOBAL PROPERTY CP_INCLUDE_DIR)

target_include_directories(${MODULE_NAME}
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CP_INCLUDE_DIR}
        ${PARSLEY_INCLUDE_DIR}
)
# ev@bcc62523-e22b-41d7-ba2f-825b493a3c97:v1

target_sources(${MODULE_NAME}
    PRIVATE
        "evse_board_support/evse_board_supportImpl.cpp"
        "temperatures/cb_temperaturesImpl.cpp"
        "mcs/cb_mcsImpl.cpp"
)

# ev@c55432ab-152c-45a9-9d2e-7281d50c69c3:v1
# insert other things like install cmds etc here

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/VERSION"
    DESTINATION "${EVEREST_MODULE_INSTALL_PREFIX}/${MODULE_NAME}"
)
# ev@c55432ab-152c-45a9-9d2e-7281d50c69c3:v1
