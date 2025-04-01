cmake_minimum_required(VERSION 3.14.7)

project(everest-chargebyte VERSION 0.20.0
    DESCRIPTION "chargebyte's Hardware EVerest Modules"
    LANGUAGES C CXX)

find_package(everest-cmake 0.4 REQUIRED
    COMPONENTS bundling
    PATHS ../everest-cmake
)

set(CMAKE_CXX_STANDARD 17)

# search for package PkgConfig
find_package(PkgConfig REQUIRED)

# search for libgpiod
pkg_search_module(LIBGPIOD REQUIRED libgpiodcxx)

# options
option(CMAKE_RUN_CLANG_TIDY "Run clang-tidy" OFF)

# dependencies
if(NOT DISABLE_EDM)
    # FIXME (aw): this implicit definition for child projects is hacky
    set(THIRD_PARTY_APP_DST "${CMAKE_INSTALL_LIBEXECDIR}/everest/3rd_party")

    evc_setup_edm()
else()
    find_package(everest-core REQUIRED)
endif()

# gpiod utils
add_subdirectory("utils")

# make custom interfaces and types visible
ev_add_project()

# config
# FIXME (aw): this should be optional
add_subdirectory(config)

# install config
install(
    FILES config/bsp-only.yaml
    DESTINATION ${CMAKE_INSTALL_DATADIR}/everest/config
    RENAME config.yaml
)

# configure clang-tidy if requested
if(CMAKE_RUN_CLANG_TIDY)
    message("Enabling clang-tidy")
    set(CMAKE_CXX_CLANG_TIDY clang-tidy)
endif()

option(BUILD_CB_TESTING "Run unit tests" OFF)
if(BUILD_CB_TESTING)
    add_subdirectory(tests)
endif()
