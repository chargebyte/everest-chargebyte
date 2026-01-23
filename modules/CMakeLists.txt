add_compile_options(-Wall -Wextra -Werror=switch)
# generated code has functions often not used
add_compile_options(-Wno-unused-function)

ev_add_cpp_module(CbChargeSOMDriver)
ev_add_cpp_module(CbCPXDriver)
ev_add_cpp_module(CbTarragonDIs)
ev_add_cpp_module(CbTarragonDriver)
ev_add_cpp_module(CbTarragonPlugLock)
ev_add_cpp_module(CbSystem)
ev_add_cpp_module(CbParsleyDriver)
ev_add_cpp_module(InfypowerDCSupply)
