add_compile_options(-Wall -Wextra -Werror=switch)
# generated code has functions often not used
add_compile_options(-Wno-unused-function)

# CbChargeSOMDriver and CbParsleyDriver depend on ra-utils and should only
# be built when the package is available in the selected sysroot.
find_package(ra-utils QUIET CONFIG)
if(TARGET ra-utils)
    ev_add_cpp_module(CbChargeSOMDriver)
    ev_add_cpp_module(CbParsleyDriver)
else()
    message(STATUS "Skipping CbChargeSOMDriver and CbParsleyDriver: ra-utils not found")
endif()

ev_add_cpp_module(CbTarragonDIs)
ev_add_cpp_module(CbTarragonDriver)
ev_add_cpp_module(CbTarragonPlugLock)
ev_add_cpp_module(CbSystem)
ev_add_cpp_module(InfypowerDCSupply)
