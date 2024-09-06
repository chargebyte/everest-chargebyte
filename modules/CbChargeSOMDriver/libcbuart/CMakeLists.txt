set(LIBCBUART_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" PARENT_SCOPE)

add_library(libcbuart STATIC)

target_sources(libcbuart
    PRIVATE
        "cb_protocol.c"
        "crc8_j1850.c"
        "logging.c"
        "tools.c"
        "uart.c"
)
