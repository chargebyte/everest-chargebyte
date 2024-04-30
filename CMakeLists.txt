cmake_minimum_required(VERSION 3.14.7)

project(chargebytetarragondriver VERSION 0.8.0
    DESCRIPTION "chargebyte's Tarragon driver for EVerest"
    LANGUAGES CXX)

find_package(everest-cmake 0.1 REQUIRED
    COMPONENTS bundling
    PATHS ../everest-cmake
)

# search for package PkgConfig
find_package(PkgConfig REQUIRED)

# search for libgpiod
pkg_search_module(LIBGPIOD REQUIRED libgpiodcxx)

# gpiod utils
add_subdirectory("utils")

# options
option(BUILD_TESTING "Run unit tests" OFF)
option(CMAKE_RUN_CLANG_TIDY "Run clang-tidy" OFF)

# dependencies
if(NOT DISABLE_EDM)
    # FIXME (aw): this implicit definition for child projects is hacky
    set(THIRD_PARTY_APP_DST "${CMAKE_INSTALL_LIBEXECDIR}/everest/3rd_party")

    evc_setup_edm()
else()
    find_package(everest-core REQUIRED)
endif()

# make custom interfaces and types visible
ev_add_project()

# config
# FIXME (aw): this should be optional
add_subdirectory(config)

# install config
install(
    FILES config/config-chargebyte-tarragon.yaml
    DESTINATION ${CMAKE_INSTALL_DATADIR}/everest/config
    RENAME config.yaml
)

# configure clang-tidy if requested
if(CMAKE_RUN_CLANG_TIDY)
    message("Enabling clang-tidy")
    set(CMAKE_CXX_CLANG_TIDY clang-tidy)
endif()

# testing
if(BUILD_TESTING)
    include(CTest)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)

    evc_include(CodeCoverage)

    append_coverage_compiler_flags()

    add_subdirectory(tests)

    setup_target_for_coverage_gcovr_html(
        NAME gcovr_coverage
        EXECUTABLE test_config
        DEPENDENCIES test_config everest
    )

    setup_target_for_coverage_lcov(
        NAME lcov_coverage
        EXECUTABLE test_config
        DEPENDENCIES test_config everest
    )
endif()
