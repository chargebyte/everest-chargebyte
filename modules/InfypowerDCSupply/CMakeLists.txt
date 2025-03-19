#
# AUTO GENERATED - MARKED REGIONS WILL BE KEPT
# template version 3
#

# module setup:
#   - ${MODULE_NAME}: module name
ev_setup_cpp_module()

# ev@bcc62523-e22b-41d7-ba2f-825b493a3c97:v1
# insert your custom targets and additional config variables here

# search for libsocketcan
pkg_search_module(LIBSOCKETCAN REQUIRED libsocketcan)

set(MODULE_DESCRIPTION "chargebyte's EVerest module for Infypower DC modules")
set(MODULE_VERSION ${PROJECT_VERSION})

# make CMake project variables usable in source code
configure_file(configuration.h.in configuration.h @ONLY)

# create a VERSION file
configure_file(VERSION.in VERSION @ONLY)

target_include_directories(${MODULE_NAME}
    PRIVATE
        # for configuration.h
        ${CMAKE_CURRENT_BINARY_DIR}
)

target_sources(${MODULE_NAME}
    PRIVATE
        "datatype_tools.c"
        "InfypowerCANID.cpp"
)

target_link_libraries(${MODULE_NAME}
    PRIVATE
        ${LIBSOCKETCAN_LIBRARIES}
)
# ev@bcc62523-e22b-41d7-ba2f-825b493a3c97:v1

target_sources(${MODULE_NAME}
    PRIVATE
        "main/power_supply_DCImpl.cpp"
)

# ev@c55432ab-152c-45a9-9d2e-7281d50c69c3:v1
# insert other things like install cmds etc here

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/VERSION"
    DESTINATION "${EVEREST_MODULE_INSTALL_PREFIX}/${MODULE_NAME}"
)
# ev@c55432ab-152c-45a9-9d2e-7281d50c69c3:v1
