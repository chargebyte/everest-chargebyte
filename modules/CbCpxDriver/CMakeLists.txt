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

add_subdirectory("cpx")

set(MODULE_DESCRIPTION "chargebyte's CPX EVerest module")
set(MODULE_VERSION ${PROJECT_VERSION})

# make CMake project variables usable in source code
configure_file(configuration.h.in configuration.h @ONLY)

# create a VERSION file
configure_file(VERSION.in VERSION @ONLY)

target_link_libraries(${MODULE_NAME}
    PRIVATE
        Pal::Sigslot
        nlohmann_json::nlohmann_json
        cpx
        utils::cp
)
target_include_directories(${MODULE_NAME}
    PRIVATE
        ${CPX_INCLUDE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/cpx
)
# ev@bcc62523-e22b-41d7-ba2f-825b493a3c97:v1

target_sources(${MODULE_NAME}
    PRIVATE
        "evse_board_support/evse_board_supportImpl.cpp"
        "temperatures/cb_temperaturesImpl.cpp"
)

# ev@c55432ab-152c-45a9-9d2e-7281d50c69c3:v1
# insert other things like install cmds etc here

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/VERSION"
    DESTINATION "${EVEREST_MODULE_INSTALL_PREFIX}/${MODULE_NAME}"
)
# ev@c55432ab-152c-45a9-9d2e-7281d50c69c3:v1
