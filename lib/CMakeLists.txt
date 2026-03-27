# chargebyte's utility/helper library

add_library(cb-utils SHARED)
add_library(chargebyte::utils ALIAS cb-utils)
ev_register_library_target(cb-utils)

target_sources(cb-utils
    PRIVATE
        src/gpiodUtils.cpp
        src/IIOADC.cpp
        src/IIOADCUtils.cpp
        src/PWMChip.cpp
        src/PWM.cpp
        src/SysfsDevice.cpp
)

target_include_directories(cb-utils
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(cb-utils
    PUBLIC
        ${LIBGPIOD_LIBRARIES}
)

set_target_properties(cb-utils
    PROPERTIES
        VERSION ${PROJECT_VERSION}
        SOVERSION 1
)

set(CBUTILS_INCLUDE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}")

install(
    DIRECTORY include/
    TYPE INCLUDE
)

install(
    TARGETS cb-utils
    EXPORT cb-utilsTargets
    LIBRARY DESTINATION lib
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT cb-utilsTargets
    FILE cb-utilsTargets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/cb-utils
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cb-utilsConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/cb-utilsConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/cb-utils
    PATH_VARS CBUTILS_INCLUDE_INSTALL_DIR
)

install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/cb-utilsConfig.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/cb-utils
)

if (BUILD_CB_TESTING)
    add_subdirectory(tests)
endif()
