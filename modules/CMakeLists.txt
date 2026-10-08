add_compile_options(-Wall -Wextra -Werror=switch)
# generated code has functions often not used
add_compile_options(-Wno-unused-function)

# CbChargeSOMDriver and CbParsleyDriver depend on ra-utils
if(everest-chargebyte_WITH_RA_UTILS)
    ev_add_cpp_module(CbChargeSOMDriver)
    ev_add_cpp_module(CbParsleyDriver)
else()
    message(STATUS "Skipping CbChargeSOMDriver and CbParsleyDriver: everest-chargebyte_WITH_RA_UTILS=OFF")
endif()

ev_add_cpp_module(CbGPIOEnergyLimits)
ev_add_cpp_module(CbGPIOSMITH)
ev_add_cpp_module(CbTarragonDIs)
ev_add_cpp_module(CbTarragonDriver)
ev_add_cpp_module(CbTarragonPlugLock)
ev_add_cpp_module(CbSystem)
ev_add_cpp_module(InfypowerDCSupply)
