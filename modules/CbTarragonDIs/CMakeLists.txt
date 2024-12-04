#
# AUTO GENERATED - MARKED REGIONS WILL BE KEPT
# template version 3
#

# module setup:
#   - ${MODULE_NAME}: module name
ev_setup_cpp_module()

# ev@bcc62523-e22b-41d7-ba2f-825b493a3c97:v1
# insert your custom targets and additional config variables here
add_subdirectory("tarragon")

set(MODULE_DESCRIPTION "chargebyte's Tarragon driver for configuration of digital input reference PWM")
set(MODULE_VERSION ${PROJECT_VERSION})

# make CMake project variables usable in source code
configure_file(configuration.h.in configuration.h @ONLY)

# create a VERSION file
configure_file(VERSION.in VERSION @ONLY)

get_property(LINUX_HW_INCLUDE_DIR GLOBAL PROPERTY LINUX_HW_INCLUDE_DIR)

target_include_directories(${MODULE_NAME}
    PRIVATE
        ${LINUX_HW_INCLUDE_DIR}
        ${TARRAGON_INCLUDE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(${MODULE_NAME}
    PRIVATE
        tarragon
        utils::linux_hw
)
# ev@bcc62523-e22b-41d7-ba2f-825b493a3c97:v1

target_sources(${MODULE_NAME}
    PRIVATE
        "empty/emptyImpl.cpp"
)

# ev@c55432ab-152c-45a9-9d2e-7281d50c69c3:v1
# insert other things like install cmds etc here

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/VERSION"
    DESTINATION "${EVEREST_MODULE_INSTALL_PREFIX}/${MODULE_NAME}"
)
# ev@c55432ab-152c-45a9-9d2e-7281d50c69c3:v1
